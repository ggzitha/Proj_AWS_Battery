import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:sqflite/sqflite.dart';
import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
  SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
    DeviceOrientation.portraitDown,
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);

  await AppDb.instance.init();
  runApp(const MyApp());
}

/// BLE UUIDs — final (not const)
final Uuid kServiceUuid = Uuid.parse("91bad492-b950-4226-aa2b-4ede9fa42f59");
final Uuid kCharUuid = Uuid.parse("cba1d466-344c-4be3-ab3f-189daa0a16d8");

/// Target device names (no MAC hardcoding)
const targetNames = {"BATT-Mon_4", "BATT-Mon_5", "BATT-Mon_6"};

/// DB helper (single-row settings)
class AppDb {
  AppDb._();
  static final AppDb instance = AppDb._();
  Database? _db;

  Future<void> init() async {
    if (_db != null) return;
    final dir = await getApplicationDocumentsDirectory();
    final path = p.join(dir.path, 'mee_bat_monitor.db');
    _db = await openDatabase(
      path,
      version: 1,
      onCreate: (db, v) async {
        await db.execute('''
          CREATE TABLE settings(
            id INTEGER PRIMARY KEY CHECK (id = 1),
            vmin REAL NOT NULL,
            vnom REAL NOT NULL,
            vmax REAL NOT NULL,
            series INTEGER NOT NULL,
            parallel INTEGER NOT NULL,
            cell_mah INTEGER NOT NULL,
            r_cell_milliohm REAL NOT NULL
          )
        ''');
        await db.insert('settings', {
          'id': 1,
          'vmin': 2.70,
          'vnom': 3.60,
          'vmax': 4.20,
          'series': 4,
          'parallel': 3,
          'cell_mah': 4000,
          'r_cell_milliohm': 18.0,
        });
      },
    );
  }

  Future<Map<String, dynamic>> loadSettings() async {
    final rows = await _db!.query('settings', where: 'id=1', limit: 1);
    if (rows.isEmpty) {
      await resetDefaults();
      return loadSettings();
    }
    return rows.first;
  }

  Future<void> saveSettings({
    required double vmin,
    required double vnom,
    required double vmax,
    required int series,
    required int parallel,
    required int cellMah,
    required double rCellMilliOhm,
  }) async {
    await _db!.update(
      'settings',
      {
        'vmin': vmin,
        'vnom': vnom,
        'vmax': vmax,
        'series': series,
        'parallel': parallel,
        'cell_mah': cellMah,
        'r_cell_milliohm': rCellMilliOhm,
      },
      where: 'id=1',
    );
  }

  Future<void> resetDefaults() async {
    await _db!.update(
      'settings',
      {
        'vmin': 2.70,
        'vnom': 3.60,
        'vmax': 4.20,
        'series': 4,
        'parallel': 3,
        'cell_mah': 4000,
        'r_cell_milliohm': 18.0,
      },
      where: 'id=1',
    );
  }
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: "Mee_BAT Monitor",
      themeMode: ThemeMode.system,
      theme: ThemeData(
        colorSchemeSeed: Colors.teal,
        useMaterial3: true,
        brightness: Brightness.light,
      ),
      darkTheme: ThemeData(
        colorSchemeSeed: Colors.teal,
        useMaterial3: true,
        brightness: Brightness.dark,
      ),
      home: const HomePage(),
    );
  }
}

class ParsedReading {
  final DateTime ts;
  final double voltx;
  final double ampx;
  final double tempx;
  double get watt => voltx * ampx;
  ParsedReading(this.ts, this.voltx, this.ampx, this.tempx);
}

/// Sliding window by time (1 jam)
class TimeSeries {
  final Duration window;
  final List<ParsedReading> _items = [];
  TimeSeries({required this.window});

  void add(ParsedReading r) {
    _items.add(r);
    prune();
  }

  void prune() {
    final cutoff = DateTime.now().subtract(window);
    while (_items.isNotEmpty && _items.first.ts.isBefore(cutoff)) {
      _items.removeAt(0);
    }
  }

  List<ParsedReading> get items {
    prune();
    return List.unmodifiable(_items);
  }

  bool get isEmpty => items.isEmpty;
}

/// Estimation bundle
class Estimate {
  final bool charging;
  final double socNow;
  final Duration? toEmpty;
  final Duration? toFull;
  final DateTime calcTime;
  final List<FlSpot> futureSeries;
  final List<FlSpot> pastSeries;

  /// Label sumber ETA di UI
  final String etaMethodLabel;

  Estimate({
    required this.charging,
    required this.socNow,
    required this.toEmpty,
    required this.toFull,
    required this.calcTime,
    required this.futureSeries,
    required this.pastSeries,
    required this.etaMethodLabel,
  });
}

class DeviceState {
  final DiscoveredDevice device;
  StreamSubscription<ConnectionStateUpdate>? connSub;
  StreamSubscription<List<int>>? notifySub;
  final TimeSeries history = TimeSeries(window: const Duration(hours: 1));
  ParsedReading? last;
  bool connected = false;
  String? error;
  bool expanded = true;
  DateTime? lastDisconnectTime;

  /// Whether user wants this device connected (for scan-auto-connect behavior)
  bool userWantsConnection = true;

  // SoC / estimate
  double _socPctEma = 0;
  DateTime? _lastSocTs;
  final List<MapEntry<DateTime, double>> socHist = [];
  int _lastEstimateLen = 0;
  Estimate? estimate;

  // Coulomb counting state
  double _accumulatedAh = 0.0;
  DateTime? _lastCCUpdate;

  // Anchor Coulomb seperti firmware (first OCV)
  bool _ccInitialized = false;
  double _ccBaseSocPct = 50.0;

  // RAW capture
  String? lastRaw;
  String? lastRawVoltx;
  String? lastRawAmpx;
  String? lastRawTempx;

  // Chart tab per device
  final TabControllerHolder tabCtl = TabControllerHolder();

  DeviceState(this.device);

  void dispose() {
    connSub?.cancel();
    notifySub?.cancel();
    tabCtl.dispose();
  }
}

class TabControllerHolder with ChangeNotifier {
  TabController? controller;
  void attach(TickerProvider vsync) {
    controller ??= TabController(length: 4, vsync: vsync);
  }

  void dispose() {
    controller?.dispose();
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});
  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> with TickerProviderStateMixin {
  final _ble = FlutterReactiveBle();
  StreamSubscription<DiscoveredDevice>? _scanSub;

  final Map<String, DeviceState> _devices = {};
  bool _scanning = false;
  Timer? _cleanupTimer;

  // Settings
  double _vMax = 4.20;
  double _vNom = 3.60;
  double _vMin = 2.70;
  int _seriesCells = 4;
  int _parallelCells = 3;
  int _cellMah = 4000;
  double _rCellMilliOhm = 18;

  static const double _emaAlphaCharge = 0.15;
  static const double _emaAlphaDischarge = 0.25;

  @override
  void initState() {
    super.initState();
    _loadSettings().then((_) => _initPermissionsThenScan());
    _startCleanupTimer();
  }

  @override
  void dispose() {
    _cleanupTimer?.cancel();
    _scanSub?.cancel();
    for (final d in _devices.values) {
      d.dispose();
    }
    super.dispose();
  }

  void _startCleanupTimer() {
    _cleanupTimer = Timer.periodic(const Duration(seconds: 5), (_) {
      _cleanupDisconnectedDevices();
    });
  }

  void _cleanupDisconnectedDevices() {
    final now = DateTime.now();
    final toRemove = <String>[];

    for (final entry in _devices.entries) {
      final ds = entry.value;
      if (!ds.connected && ds.lastDisconnectTime != null) {
        final disconnectedFor = now.difference(ds.lastDisconnectTime!);
        if (disconnectedFor.inSeconds > 30) {
          toRemove.add(entry.key);
        }
      }
    }

    if (toRemove.isNotEmpty) {
      setState(() {
        for (final id in toRemove) {
          _devices[id]?.dispose();
          _devices.remove(id);
        }
      });
      if (toRemove.length == 1) {
        _snack("Perangkat yang terputus >30s telah dihapus");
      } else {
        _snack("${toRemove.length} perangkat yang terputus >30s telah dihapus");
      }
    }
  }

  Future<void> _loadSettings() async {
    final m = await AppDb.instance.loadSettings();
    setState(() {
      _vMin = (m['vmin'] as num).toDouble();
      _vNom = (m['vnom'] as num).toDouble();
      _vMax = (m['vmax'] as num).toDouble();
      _seriesCells = (m['series'] as num).toInt().clamp(1, 20);
      _parallelCells = (m['parallel'] as num).toInt().clamp(1, 50);
      _cellMah = (m['cell_mah'] as num).toInt().clamp(500, 100000);
      _rCellMilliOhm = (m['r_cell_milliohm'] as num).toDouble().clamp(0, 1000);
      if (!(_vMin < _vNom && _vNom < _vMax)) {
        _vMin = 2.70;
        _vNom = 3.60;
        _vMax = 4.20;
      }
    });
  }

  Future<void> _saveSettings({
    required double vmax,
    required double vnom,
    required double vmin,
    required int series,
    required int parallel,
    required int cellMah,
    required double rMilliOhm,
  }) async {
    if (!(vmin < vnom && vnom < vmax)) {
      _snack("Pengaturan Tegangan Error: Nilai Min < Nom < Max !!");
      return;
    }
    await AppDb.instance.saveSettings(
      vmin: vmin,
      vnom: vnom,
      vmax: vmax,
      series: series,
      parallel: parallel,
      cellMah: cellMah,
      rCellMilliOhm: rMilliOhm,
    );
    setState(() {
      _vMax = vmax;
      _vNom = vnom;
      _vMin = vmin;
      _seriesCells = series;
      _parallelCells = parallel;
      _cellMah = cellMah;
      _rCellMilliOhm = rMilliOhm;
    });
    _snack("Pengaturan disimpan.");
  }

  Future<void> _resetDefaults() async {
    await AppDb.instance.resetDefaults();
    await _loadSettings();
    _snack("Pengaturan dikembalikan ke default.");
  }

  Future<void> _initPermissionsThenScan() async {
    final statuses = await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.locationWhenInUse,
    ].request();

    final allGranted = statuses.values.every((s) => s.isGranted);
    if (!allGranted && mounted) {
      _snack("Wajib Memberi Akses Izin Untuk BLE & Lokasi.");
    }
    _startScan();
  }

  void _startScan() {
    if (_scanning) return;
    setState(() => _scanning = true);

    _scanSub?.cancel();
    _scanSub = null;

    try {
      _scanSub = _ble.scanForDevices(
          withServices: const [], scanMode: ScanMode.lowLatency).listen((d) {
        if (!targetNames.contains(d.name)) return;

        if (!_devices.containsKey(d.id)) {
          final ds = DeviceState(d);
          ds.tabCtl.attach(this);
          ds.userWantsConnection = true; // auto-connect first discovery
          _devices[d.id] = ds;
          setState(() {});
          _connectAndSubscribe(ds);
        } else {
          final ds = _devices[d.id]!;
          if (!ds.connected && ds.userWantsConnection) {
            _connectAndSubscribe(ds);
          }
        }
      }, onError: (e) {
        _snack("Scan error: $e");
      });
    } catch (e) {
      _snack("Scan error: $e");
      setState(() => _scanning = false);
    }
  }

  void _stopScan() {
    _scanSub?.cancel();
    _scanSub = null;
    setState(() => _scanning = false);
  }

  void _rescan() {
    _stopScan();
    Future.delayed(const Duration(milliseconds: 100), () {
      _startScan();
    });
  }

  Future<void> _connectAndSubscribe(DeviceState ds) async {
    if (!ds.userWantsConnection) return;
    if (ds.connected) return;

    ds.error = null;
    ds.lastDisconnectTime = null;
    await ds.connSub?.cancel();

    try {
      ds.connSub = _ble
          .connectToDevice(
        id: ds.device.id,
        connectionTimeout: const Duration(seconds: 10),
      )
          .listen((update) {
        if (!mounted) return;
        switch (update.connectionState) {
          case DeviceConnectionState.connected:
            ds.connected = true;
            ds.lastDisconnectTime = null;
            setState(() {});
            _ble
                .requestMtu(deviceId: ds.device.id, mtu: 256)
                .catchError((_) {});
            _subscribe(ds);
            break;
          case DeviceConnectionState.disconnected:
            ds.connected = false;
            ds.lastDisconnectTime = DateTime.now();
            setState(() {});
            break;
          default:
            break;
        }
      }, onError: (e) {
        ds.error = "Gagal Menghubungkan: $e";
        ds.connected = false;
        ds.lastDisconnectTime = DateTime.now();
        setState(() {});
      });
    } catch (e) {
      ds.error = "Gagal Menghubungkan: $e";
      ds.connected = false;
      ds.lastDisconnectTime = DateTime.now();
      setState(() {});
    }
  }

  Map<String, String> _extractRawTokens(String raw) {
    final out = <String, String>{};
    String normNum(String s) {
      s = s.trim();
      if (s.endsWith('.')) s = '${s}0';
      return s;
    }

    final kv = RegExp(r'\b([a-zA-Z]+)\s*=\s*(-?\d+(?:\.\d*)?)\b',
        caseSensitive: false);
    for (final m in kv.allMatches(raw)) {
      final k = m.group(1)!.toLowerCase();
      final v = normNum(m.group(2)!);
      out.putIfAbsent(k, () => v);
    }

    final compact = RegExp(r'([VvAaTtHh])\s*(-?\d+(?:\.\d*)?)');
    for (final m in compact.allMatches(raw)) {
      final k = m.group(1)!.toLowerCase();
      if (k == 'v' || k == 'a' || k == 't' || k == 'h') {
        final v = normNum(m.group(2)!);
        out.putIfAbsent(k, () => v);
      }
    }
    return out;
  }

  ParsedReading _parsePayloadFromTokens(Map<String, String> toks) {
    double? pickD(List<String> keys) {
      for (final k in keys) {
        final s = toks[k];
        if (s == null) continue;
        final d = double.tryParse(s);
        if (d != null) return d;
      }
      return null;
    }

    final voltx = pickD(['voltx', 'voltage', 'v']);
    final ampx = pickD(['ampx', 'ampere', 'a']);
    final tempx = pickD(['tempx', 'temperature', 't']);

    if (tempx == null || ampx == null || voltx == null) {
      throw FormatException("Data tidak valid (token hilang): $toks");
    }
    return ParsedReading(DateTime.now(), voltx, ampx, tempx);
  }

  void _subscribe(DeviceState ds) {
    ds.notifySub?.cancel();

    final ch = QualifiedCharacteristic(
      deviceId: ds.device.id,
      serviceId: kServiceUuid,
      characteristicId: kCharUuid,
    );

    ds.notifySub = _ble.subscribeToCharacteristic(ch).listen((data) {
      try {
        final raw = String.fromCharCodes(data)
            .replaceAll(RegExp(r'[\x00-\x1F]'), ' ')
            .trim();
        ds.lastRaw = raw;

        final toks = _extractRawTokens(raw);
        ds.lastRawVoltx = toks['voltx'] ?? toks['voltage'] ?? toks['v'];
        ds.lastRawAmpx = toks['ampx'] ?? toks['ampere'] ?? toks['a'];
        ds.lastRawTempx = toks['tempx'] ?? toks['temperature'] ?? toks['t'];

        final reading = _parsePayloadFromTokens(toks);
        ds.last = reading;
        ds.history.add(reading);

        // Update Coulomb Counting
        _updateCoulombCounting(ds, reading);

        final isCharging = reading.ampx > 0.0;

        // ============================================================
        // SOC dari OCV + IR compensation
        // ============================================================
        final socOCV = _socPercentOCV(reading);

        // Coulomb anchored ke OCV
        final socCC = _socPercentCC(ds, socOCV);

        // Gabung OCV + CC
        final socCombinedRaw = (socOCV * 0.85 + socCC * 0.15).clamp(0.0, 100.0);

        ds._socPctEma = _emaNext(
          ds._socPctEma,
          socCombinedRaw,
          (isCharging ? _emaAlphaCharge : _emaAlphaDischarge),
        );
        ds._lastSocTs = reading.ts;

        // Simpan history SoC untuk slope / ETA
        ds.socHist.add(MapEntry(reading.ts, ds._socPctEma));
        final cutoff = DateTime.now().subtract(const Duration(minutes: 15));
        while (ds.socHist.isNotEmpty && ds.socHist.first.key.isBefore(cutoff)) {
          ds.socHist.removeAt(0);
        }

        if (ds.socHist.length >= 5 &&
            (ds.socHist.length - ds._lastEstimateLen) >= 5) {
          ds.estimate = _computeEstimate(ds);
          ds._lastEstimateLen = ds.socHist.length;
        }

        if (mounted) setState(() {});
      } catch (e) {
        ds.error = "Parsing error: $e";
        if (mounted) setState(() {});
      }
    }, onError: (e) {
      ds.error = "Notifikasi error: $e";
      if (mounted) setState(() {});
    });
  }

  void _updateCoulombCounting(DeviceState ds, ParsedReading reading) {
    if (ds._lastCCUpdate == null) {
      ds._lastCCUpdate = reading.ts;
      return;
    }

    final dtSec =
        reading.ts.difference(ds._lastCCUpdate!).inMilliseconds / 1000.0;
    if (dtSec <= 0) return;

    final deltaAh = (reading.ampx * dtSec) / 3600.0;
    ds._accumulatedAh += deltaAh;
    ds._lastCCUpdate = reading.ts;
  }

  double _socPercentCC(DeviceState ds, double socOcvNow) {
    final capAh = (_cellMah / 1000.0) * _parallelCells;
    if (capAh <= 0) return socOcvNow.clamp(0.0, 100.0);

    if (!ds._ccInitialized) {
      ds._ccInitialized = true;
      ds._ccBaseSocPct = socOcvNow.clamp(0.0, 100.0);
      ds._accumulatedAh = 0.0;
      return ds._ccBaseSocPct;
    }

    final socDelta = (ds._accumulatedAh / capAh) * 100.0;
    return (ds._ccBaseSocPct + socDelta).clamp(0.0, 100.0);
  }

  // SOC berdasar OCV + IR compensation
  double _socPercentOCV(ParsedReading r) {
    final rPerCellMilliOhm = _rCellMilliOhm.clamp(0, 1000);

    // paralel
    final safeParallel = _parallelCells <= 0 ? 1 : _parallelCells;
    final rGroupMilliOhm = rPerCellMilliOhm / safeParallel;

    // seri
    final rSeriesMilliOhm = rGroupMilliOhm * _seriesCells;

    // extra (BMS FET, kabel, dsb)
    const rExtraMilliOhm = 60.0;
    final rPackTotalMilliOhm = rSeriesMilliOhm + rExtraMilliOhm;
    final rPackOhm = rPackTotalMilliOhm / 1000.0;

    final isCharging = r.ampx > 0.0;
    final vRestPack = isCharging
        ? (r.voltx - r.ampx * rPackOhm)
        : (r.voltx + r.ampx.abs() * rPackOhm);

    final vCell = (vRestPack / _seriesCells).clamp(0.0, 1000.0);

    final vmin = _vMin;
    final vnom = _vNom;
    final vmax = _vMax;

    double pct;
    if (vCell <= vmin) {
      pct = 0.0;
    } else if (vCell >= vmax) {
      pct = 100.0;
    } else if (vCell <= vnom) {
      pct = 50.0 * ((vCell - vmin) / (vnom - vmin));
    } else {
      pct = 50.0 + 50.0 * ((vCell - vnom) / (vmax - vnom));
    }
    return pct.clamp(0.0, 100.0);
  }

  Estimate? _computeEstimate(DeviceState ds) {
    final hist = ds.socHist;
    if (hist.length < 5) {
      return ds.estimate;
    }

    // Ambil ~5 menit terakhir
    final newestTime = hist.last.key;
    final oldestAllowed = newestTime.subtract(const Duration(minutes: 5));
    final recent = hist.where((e) => !e.key.isBefore(oldestAllowed)).toList();

    if (recent.length < 3) {
      return ds.estimate;
    }

    final t0 = recent.first.key;
    final xs = <double>[]; // detik sejak t0
    final ys = <double>[]; // SoC %

    for (final e in recent) {
      final dtSec = e.key.difference(t0).inMilliseconds / 1000.0;
      xs.add(dtSec);
      ys.add(e.value);
    }

    final n = xs.length;
    final totalSpanSec = xs.last;
    if (totalSpanSec < 30) {
      return ds.estimate;
    }

    // Linear regression
    final meanX = xs.reduce((a, b) => a + b) / n;
    final meanY = ys.reduce((a, b) => a + b) / n;

    double num = 0;
    double den = 0;
    for (int i = 0; i < n; i++) {
      final dx = xs[i] - meanX;
      num += dx * (ys[i] - meanY);
      den += dx * dx;
    }
    if (den == 0) {
      return ds.estimate;
    }

    final slope = num / den; // % per detik
    final sN = ys.last; // SoC % terakhir (smoothed)
    final tN = t0.add(Duration(milliseconds: (xs.last * 1000).round()));

    final I = (ds.last?.ampx ?? 0.0);
    final absI = I.abs();
    final isChargingBySign = I > 0.0;

    final capAh = (_cellMah / 1000.0) * _parallelCells;

    // threshold
    const double minSlope = 5e-7;
    const double minCurrA = 0.005;
    const double currPriorityThresh = 0.01; // >=10mA => utamakan arus

    Duration? etaSlope;
    Duration? etaCurrent;

    // --- ETA dari slope SoC ---
    if (isChargingBySign) {
      if (slope > minSlope) {
        final secondsToFull =
            ((100.0 - sN) / slope).clamp(0, 365.0 * 24 * 3600).toDouble();
        etaSlope = Duration(seconds: secondsToFull.round());
      }
    } else {
      if (slope < -minSlope) {
        final secondsToEmpty =
            (sN / (-slope)).clamp(0, 365.0 * 24 * 3600).toDouble();
        etaSlope = Duration(seconds: secondsToEmpty.round());
      }
    }

    // --- ETA dari arus + kapasitas ---
    if (absI > minCurrA && capAh > 0) {
      final pctPerSec = (absI / (capAh * 3600.0)) * 100.0;
      if (pctPerSec > 0) {
        if (isChargingBySign) {
          final secondsToFull =
              ((100.0 - sN) / pctPerSec).clamp(0, 365.0 * 24 * 3600).toDouble();
          etaCurrent = Duration(seconds: secondsToFull.round());
        } else {
          final secondsToEmpty =
              (sN / pctPerSec).clamp(0, 365.0 * 24 * 3600).toDouble();
          etaCurrent = Duration(seconds: secondsToEmpty.round());
        }
      }
    }

    Duration? toFull;
    Duration? toEmpty;
    Duration? chosenEta;
    String methodLabel = "";

    if (isChargingBySign) {
      if (absI >= currPriorityThresh && etaCurrent != null) {
        toFull = etaCurrent;
        chosenEta = etaCurrent;
        methodLabel = "Berdasarkan arus & kapasitas";
      } else if (etaSlope != null) {
        toFull = etaSlope;
        chosenEta = etaSlope;
        methodLabel = "Berdasarkan tren SoC";
      } else if (etaCurrent != null) {
        toFull = etaCurrent;
        chosenEta = etaCurrent;
        methodLabel = "Berdasarkan arus & kapasitas";
      }
    } else {
      if (absI >= currPriorityThresh && etaCurrent != null) {
        toEmpty = etaCurrent;
        chosenEta = etaCurrent;
        methodLabel = "Berdasarkan arus & kapasitas";
      } else if (etaSlope != null) {
        toEmpty = etaSlope;
        chosenEta = etaSlope;
        methodLabel = "Berdasarkan tren SoC";
      } else if (etaCurrent != null) {
        toEmpty = etaCurrent;
        chosenEta = etaCurrent;
        methodLabel = "Berdasarkan arus & kapasitas";
      }
    }

    // validasi ringan
    if (isChargingBySign) {
      if (toFull == null || toFull.isNegative) {
        return ds.estimate;
      }
    } else {
      if (toEmpty == null || toEmpty.isNegative) {
        return ds.estimate;
      }
    }

    // data untuk line (masih dipakai di perhitungan bar)
    final past = hist
        .map((e) => FlSpot(e.key.millisecondsSinceEpoch.toDouble(), e.value))
        .toList();

    double slopeUse;
    if (chosenEta != null && chosenEta.inSeconds > 0) {
      final durSec = chosenEta.inSeconds.toDouble();
      if (isChargingBySign) {
        slopeUse = (100.0 - sN) / durSec;
      } else {
        slopeUse = -sN / durSec;
      }
    } else {
      slopeUse = slope;
    }

    final now = tN;
    final List<FlSpot> future = [];

    const int limitSec = 6 * 3600;
    final totalSec = isChargingBySign
        ? min(limitSec, (toFull?.inSeconds ?? 0))
        : min(limitSec, (toEmpty?.inSeconds ?? 0));

    if (totalSec > 0) {
      for (int s = 0; s <= totalSec; s += 10) {
        final soc = (sN + slopeUse * s).clamp(0.0, 100.0);
        future.add(
          FlSpot(
            now.add(Duration(seconds: s)).millisecondsSinceEpoch.toDouble(),
            soc,
          ),
        );
      }
    }

    // print(
    //     'ETA Debug: I=${I.toStringAsFixed(3)}A, absI=${absI.toStringAsFixed(3)}A, '
    //         'slope=${slope.toStringAsExponential(2)}, charging=$isChargingBySign, '
    //         'etaCur=${etaCurrent?.inMinutes}, etaSlope=${etaSlope?.inMinutes}, method=$methodLabel');

    return Estimate(
      charging: isChargingBySign,
      socNow: sN.clamp(0.0, 100.0),
      toEmpty: toEmpty,
      toFull: toFull,
      calcTime: now,
      futureSeries: future,
      pastSeries: past,
      etaMethodLabel: methodLabel,
    );
  }

  double _emaNext(double prev, double next, double alpha) {
    double clamp01(double v) => v < 0 ? 0 : (v > 100 ? 100 : v);
    if (prev == 0) {
      return clamp01(next);
    }
    final v = prev + alpha * (next - prev);
    return clamp01(v);
  }

  Future<void> _toggleConnection(DeviceState ds) async {
    if (ds.connected) {
      ds.userWantsConnection = false;
      ds.error = null;

      await ds.notifySub?.cancel();
      await ds.connSub?.cancel();

      ds.connected = false;
      ds.lastDisconnectTime = DateTime.now();

      setState(() {});
    } else {
      ds.userWantsConnection = true;

      await ds.connSub?.cancel();
      ds.connSub = null;

      _connectAndSubscribe(ds);
    }
  }

  void _snack(String msg) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  void _openSettingsSheet() {
    final vmaxCtl = TextEditingController(text: _vMax.toStringAsFixed(2));
    final vnomCtl = TextEditingController(text: _vNom.toStringAsFixed(2));
    final vminCtl = TextEditingController(text: _vMin.toStringAsFixed(2));
    final rCellCtl =
        TextEditingController(text: _rCellMilliOhm.toStringAsFixed(1));
    final capCtl = TextEditingController(text: _cellMah.toString());
    int seriesTmp = _seriesCells;
    int parallelTmp = _parallelCells;

    const fieldHeight = 56.0;
    const gap8 = SizedBox(width: 8);
    final captionStyle = Theme.of(context).textTheme.bodySmall?.copyWith(
          color: Theme.of(context).colorScheme.onSurfaceVariant,
        );

    showModalBottomSheet(
      context: context,
      useSafeArea: true,
      showDragHandle: true,
      isScrollControlled: true,
      builder: (ctx) => DraggableScrollableSheet(
        expand: false,
        minChildSize: 0.35,
        initialChildSize: 0.70,
        maxChildSize: 0.95,
        builder: (_, controller) => SingleChildScrollView(
          controller: controller,
          padding: EdgeInsets.only(
            left: 16,
            right: 16,
            top: 8,
            bottom: 16 + MediaQuery.of(ctx).viewInsets.bottom,
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text("Pengaturan Baterai",
                  style: Theme.of(ctx).textTheme.titleLarge),
              const SizedBox(height: 10),
              Row(
                children: [
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: TextField(
                        controller: vminCtl,
                        keyboardType: const TextInputType.numberWithOptions(
                            decimal: true),
                        decoration: const InputDecoration(
                          labelText: "Min-Voltage (V/cell)",
                          prefixIcon: Icon(Icons.battery_alert),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                  gap8,
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: TextField(
                        controller: vnomCtl,
                        keyboardType: const TextInputType.numberWithOptions(
                            decimal: true),
                        decoration: const InputDecoration(
                          labelText: "Nom-Volt (V/cell)",
                          prefixIcon: Icon(Icons.battery_charging_full),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                  gap8,
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: TextField(
                        controller: vmaxCtl,
                        keyboardType: const TextInputType.numberWithOptions(
                            decimal: true),
                        decoration: const InputDecoration(
                          labelText: "Max-Volt (V/cell)",
                          prefixIcon: Icon(Icons.bolt),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 6),
              Row(
                children: [
                  Expanded(child: Text("Default 2.70", style: captionStyle)),
                  Expanded(
                      child: Center(
                          child: Text("Default 3.60", style: captionStyle))),
                  Expanded(
                      child: Align(
                          alignment: Alignment.centerRight,
                          child: Text("Default 4.20", style: captionStyle))),
                ],
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: TextField(
                        controller: capCtl,
                        keyboardType: TextInputType.number,
                        decoration: const InputDecoration(
                          labelText: "Capacity per cell (mAh)",
                          prefixIcon: Icon(Icons.battery_saver),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                  gap8,
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: DropdownButtonFormField<int>(
                        value: seriesTmp,
                        onChanged: (v) {
                          if (v != null) {
                            seriesTmp = v;
                            (ctx as Element).markNeedsBuild();
                          }
                        },
                        items: [for (int i = 1; i <= 20; i++) i]
                            .map((e) => DropdownMenuItem(
                                value: e, child: Text("${e}s")))
                            .toList(),
                        isDense: true,
                        decoration: const InputDecoration(
                          labelText: "Series",
                          prefixIcon: Icon(Icons.grid_on),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                  gap8,
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: DropdownButtonFormField<int>(
                        value: parallelTmp,
                        onChanged: (v) {
                          if (v != null) {
                            parallelTmp = v;
                            (ctx as Element).markNeedsBuild();
                          }
                        },
                        items: [for (int i = 1; i <= 50; i++) i]
                            .map((e) => DropdownMenuItem(
                                value: e, child: Text("${e}p")))
                            .toList(),
                        isDense: true,
                        decoration: const InputDecoration(
                          labelText: "Parallel",
                          prefixIcon: Icon(Icons.view_column),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 6),
              Row(
                children: [
                  Expanded(child: Text("Default 4000", style: captionStyle)),
                  Expanded(
                      child: Center(
                          child: Text("Default 4s", style: captionStyle))),
                  Expanded(
                      child: Align(
                          alignment: Alignment.centerRight,
                          child: Text("Default 3p", style: captionStyle))),
                ],
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: SizedBox(
                      height: fieldHeight,
                      child: TextField(
                        controller: rCellCtl,
                        keyboardType: const TextInputType.numberWithOptions(
                            decimal: true),
                        decoration: const InputDecoration(
                          labelText: "Internal Resistance (mOhm)",
                          prefixIcon: Icon(Icons.speed),
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                              horizontal: 12, vertical: 16),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 6),
              Row(
                children: [
                  Expanded(child: Text("Default 18.0", style: captionStyle)),
                ],
              ),
              const SizedBox(height: 16),
              Row(
                children: [
                  OutlinedButton.icon(
                    onPressed: () async {
                      await _resetDefaults();
                      if (ctx.mounted) Navigator.pop(ctx);
                    },
                    icon: const Icon(Icons.restore),
                    label: const Text("Reset ke Default"),
                  ),
                  const Spacer(),
                  FilledButton.icon(
                    onPressed: () async {
                      final vmax = double.tryParse(vmaxCtl.text.trim());
                      final vnom = double.tryParse(vnomCtl.text.trim());
                      final vmin = double.tryParse(vminCtl.text.trim());
                      final rMilli = double.tryParse(rCellCtl.text.trim());
                      final cellMah = int.tryParse(capCtl.text.trim());
                      if (vmax == null || vnom == null || vmin == null) {
                        _snack("Masukkan nilai tegangan yang valid.");
                        return;
                      }
                      if (!(vmin < vnom && vnom < vmax)) {
                        _snack("Nilai V harus: Min < Nom < Max.");
                        return;
                      }
                      if (rMilli == null || rMilli < 0 || rMilli > 1000) {
                        _snack("Nilai resistansi tidak valid (0–1000 mOhm).");
                        return;
                      }
                      if (cellMah == null ||
                          cellMah < 500 ||
                          cellMah > 100000) {
                        _snack(
                            "Capacity per cell (mAh) tidak valid (500–100000).");
                        return;
                      }

                      await _saveSettings(
                        vmax: vmax,
                        vnom: vnom,
                        vmin: vmin,
                        series: seriesTmp,
                        parallel: parallelTmp,
                        cellMah: cellMah,
                        rMilliOhm: rMilli,
                      );
                      if (ctx.mounted) Navigator.pop(ctx);
                    },
                    icon: const Icon(Icons.save),
                    label: const Text("Simpan"),
                  ),
                ],
              ),
              const SizedBox(height: 12),
            ],
          ),
        ),
      ),
    );
  }

  EdgeInsets _screenInsets(BuildContext context) {
    final mq = MediaQuery.of(context);
    final bottomNav = mq.viewPadding.bottom;
    final gesture = mq.systemGestureInsets.bottom;
    final bottom = bottomNav > 0 ? bottomNav : min(12.0, gesture);
    return EdgeInsets.only(
      left: max(mq.viewPadding.left, mq.systemGestureInsets.left),
      right: max(mq.viewPadding.right, mq.systemGestureInsets.right),
      bottom: bottom,
      top: mq.viewPadding.top,
    );
  }

  @override
  Widget build(BuildContext context) {
    final devices = _devices.values.toList()
      ..sort((a, b) => (a.device.name).compareTo(b.device.name));

    return Scaffold(
      extendBody: true,
      appBar: AppBar(
        title: const Text("Mee_BAT Monitor"),
        actions: [
          Tooltip(
            message: "Pengaturan BMS (Volt/Series/Parallel/Cap/Resistance)",
            child: IconButton(
              onPressed: _openSettingsSheet,
              icon: const Icon(Icons.settings),
            ),
          ),
          Tooltip(
            message: "Scan ulang perangkat",
            child: IconButton(
              onPressed: _rescan,
              icon: const Icon(Icons.refresh),
            ),
          ),
          Tooltip(
            message: _scanning ? "Stop scanning" : "Start scanning",
            child: IconButton(
              onPressed: _scanning ? _stopScan : _startScan,
              icon: Icon(_scanning ? Icons.stop : Icons.search),
            ),
          ),
        ],
      ),
      body: Padding(
        padding: _screenInsets(context),
        child: devices.isEmpty
            ? const Center(child: Text("Mencari Perangkat BATT-Mon ..."))
            : ListView.builder(
                itemCount: devices.length,
                itemBuilder: (_, i) {
                  final ds = devices[i];
                  final soc = ds._socPctEma;
                  return _DeviceAccordion(
                    key: ValueKey(ds.device.id),
                    state: ds,
                    socPercent: soc,
                    onToggleConnection: () => _toggleConnection(ds),
                    vsync: this,
                  );
                },
              ),
      ),
    );
  }
}

class _DeviceAccordion extends StatelessWidget {
  const _DeviceAccordion({
    super.key,
    required this.state,
    required this.socPercent,
    required this.onToggleConnection,
    required this.vsync,
  });

  final DeviceState state;
  final double socPercent;
  final VoidCallback onToggleConnection;
  final TickerProvider vsync;

  @override
  Widget build(BuildContext context) {
    state.tabCtl.attach(vsync);

    final last = state.last;
    final title =
        state.device.name.isNotEmpty ? state.device.name : state.device.id;

    final media = MediaQuery.of(context);
    final isPortrait = media.orientation == Orientation.portrait;
    final aspectRatio = media.size.width / media.size.height;

    final double contentHeightFactor;
    if (aspectRatio > 0.5) {
      contentHeightFactor = 0.50;
    } else {
      contentHeightFactor = 0.65;
    }
    final contentHeightPortrait = media.size.height * contentHeightFactor;

    final metricPane = KeyedSubtree(
      key: ValueKey(
          "metric-${state.device.id}-${state.last?.ts.millisecondsSinceEpoch ?? 0}"),
      child: _MetricPane(
        last: last,
        rawVoltx: state.lastRawVoltx,
        rawAmpx: state.lastRawAmpx,
        rawTempx: state.lastRawTempx,
      ),
    );

    final estimateCard = _EstimateCard(state: state);
    final chartPane = KeyedSubtree(
      key: ValueKey("chart-${state.device.id}"),
      child: _ChartPane(state: state, controller: state.tabCtl.controller!),
    );

    final content = isPortrait
        ? ConstrainedBox(
            constraints: BoxConstraints(maxHeight: contentHeightPortrait),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                metricPane,
                estimateCard,
                const SizedBox(height: 4),
                Flexible(child: chartPane),
              ],
            ),
          )
        : SizedBox(
            height: 400,
            child: Row(
              children: [
                Expanded(
                  flex: 3,
                  child: Column(
                    children: [
                      metricPane,
                      estimateCard,
                      const SizedBox(height: 4),
                      Expanded(child: chartPane),
                    ],
                  ),
                ),
                const SizedBox(width: 12),
              ],
            ),
          );

    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      elevation: 2,
      child: Theme(
        data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
        child: ExpansionTile(
          initiallyExpanded: state.expanded,
          onExpansionChanged: (_) => state.expanded = !state.expanded,
          title: Row(
            children: [
              Icon(
                state.connected ? Icons.link : Icons.link_off,
                color: state.connected ? Colors.teal : Colors.grey,
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  title,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: Theme.of(context).textTheme.titleMedium,
                ),
              ),
              Flexible(
                fit: FlexFit.loose,
                child: Align(
                  alignment: Alignment.centerRight,
                  child: _BatteryBar(percent: socPercent, compact: true),
                ),
              ),
            ],
          ),
          childrenPadding: const EdgeInsets.fromLTRB(16, 0, 16, 16),
          children: [
            if (state.error != null)
              Container(
                width: double.infinity,
                padding: const EdgeInsets.all(10),
                margin: const EdgeInsets.only(bottom: 10),
                decoration: BoxDecoration(
                  color: Colors.red.withValues(alpha: .08),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Text(state.error!,
                    style: const TextStyle(color: Colors.redAccent)),
              ),
            Padding(
              padding: const EdgeInsets.only(bottom: 2.0),
              child: Align(
                alignment: Alignment.centerLeft,
                child: Tooltip(
                  message: state.connected
                      ? "Putuskan Koneksi BLE"
                      : "Hubungkan BLE",
                  child: IconButton.filledTonal(
                    onPressed: onToggleConnection,
                    icon: Icon(state.connected ? Icons.link_off : Icons.link),
                    iconSize: 28,
                  ),
                ),
              ),
            ),
            content,
          ],
        ),
      ),
    );
  }
}

class _BatteryBar extends StatefulWidget {
  const _BatteryBar({
    required this.percent,
    this.compact = true,
  });

  final double percent;
  final bool compact;

  @override
  State<_BatteryBar> createState() => _BatteryBarState();
}

class _BatteryBarState extends State<_BatteryBar>
    with SingleTickerProviderStateMixin {
  late AnimationController _ac;
  late Animation<double> _anim;
  double _fraction = 0.0;

  @override
  void initState() {
    super.initState();
    _fraction = (widget.percent.clamp(0.0, 100.0)) / 100.0;
    _ac = AnimationController(
        vsync: this, duration: const Duration(milliseconds: 160));
    _anim = AlwaysStoppedAnimation(_fraction);
  }

  @override
  void didUpdateWidget(covariant _BatteryBar oldWidget) {
    super.didUpdateWidget(oldWidget);
    final target = (widget.percent.clamp(0.0, 100.0)) / 100.0;
    if ((target - _fraction).abs() < 0.001) return;
    _ac.stop();
    _anim = Tween<double>(begin: _fraction, end: target)
        .animate(CurvedAnimation(parent: _ac, curve: Curves.easeOutCubic))
      ..addListener(() => setState(() => _fraction = _anim.value));
    _ac.forward(from: 0);
  }

  @override
  void dispose() {
    _ac.dispose();
    super.dispose();
  }

  Color _colorFor(double p) {
    p *= 100.0;
    if (p < 25) {
      return _lerpColor(
          const Color(0xFFE53935), const Color(0xFFFF7043), p / 25);
    }
    if (p < 40) {
      return _lerpColor(
          const Color(0xFFFF7043), const Color(0xFFFFEE58), (p - 25) / 15);
    }
    if (p < 60) {
      return _lerpColor(
          const Color(0xFFFFEE58), const Color(0xFF26C6DA), (p - 40) / 20);
    }
    if (p < 75) {
      return _lerpColor(
          const Color(0xFF26C6DA), const Color(0xFF66BB6A), (p - 60) / 15);
    }
    return const Color(0xFF43A047);
  }

  static Color _lerpColor(Color a, Color b, double t) =>
      Color.lerp(a, b, t.clamp(0.0, 1.0))!;
  Color _bestTextColor(Color bg) =>
      bg.computeLuminance() > 0.55 ? Colors.black : Colors.white;

  @override
  Widget build(BuildContext context) {
    final trackColor = Colors.grey;
    final textPct = (_fraction * 100.0).clamp(0.0, 100.0);
    final text = "${textPct.toStringAsFixed(0)}%";
    final fillColor = _colorFor(_fraction);

    final height = 18.0;
    final radius = 5.0;
    final fontSize = 14.0;
    final fontWeight = FontWeight.w900;

    final bar = Container(
      height: height,
      decoration: BoxDecoration(
        color: trackColor,
        borderRadius: BorderRadius.circular(radius),
        border: Border.all(color: Colors.grey.shade400, width: 1),
      ),
      clipBehavior: Clip.hardEdge,
      child: Stack(
        children: [
          FractionallySizedBox(
              widthFactor: _fraction, child: Container(color: fillColor)),
          Center(
            child: Text(
              text,
              style: TextStyle(
                fontWeight: fontWeight,
                fontSize: fontSize,
                color:
                    _fraction < 0.15 ? Colors.black : _bestTextColor(fillColor),
              ),
            ),
          ),
        ],
      ),
    );

    return ConstrainedBox(
        constraints: const BoxConstraints(minWidth: 120, maxWidth: 160),
        child: bar);
  }
}

class _MetricPane extends StatelessWidget {
  const _MetricPane(
      {required this.last, this.rawVoltx, this.rawAmpx, this.rawTempx});
  final ParsedReading? last;

  final String? rawVoltx;
  final String? rawAmpx;
  final String? rawTempx;

  String _showRawOrDouble(String? raw, double? num, {int maxDecimals = 2}) {
    final validRaw =
        (raw != null && RegExp(r'^-?\d+(?:\.\d+)?$').hasMatch(raw));
    if (validRaw) return raw;
    if (num == null) return "—";
    String s = num.toStringAsFixed(maxDecimals);
    s = s.replaceFirst(RegExp(r'\.?0+$'), '');
    return s;
  }

  @override
  Widget build(BuildContext context) {
    if (last == null) {
      return const Align(
        alignment: Alignment.centerLeft,
        child: Padding(
          padding: EdgeInsets.symmetric(vertical: 8),
          child: Text("Menunggu Data..."),
        ),
      );
    }
    final r = last!;

    final tiles = [
      _MetricTile(
          label: "Tegangan",
          value: "${_showRawOrDouble(rawVoltx, r.voltx, maxDecimals: 2)} V"),
      _MetricTile(
          label: "Arus",
          value: "${_showRawOrDouble(rawAmpx, r.ampx, maxDecimals: 2)} A"),
      _MetricTile(
          label: "Suhu",
          value: "${_showRawOrDouble(rawTempx, r.tempx, maxDecimals: 2)} °C"),
      _MetricTile(
          label: "Daya",
          value:
              "${(r.voltx * r.ampx).toStringAsFixed(2).replaceFirst(RegExp(r'\.?0+$'), '')} W"),
    ];

    return LayoutBuilder(builder: (context, c) {
      const crossAxisCount = 2;
      const extent = 64.0;
      return GridView.builder(
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        itemCount: tiles.length,
        gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: crossAxisCount,
          childAspectRatio: 3.8,
          mainAxisExtent: extent,
          crossAxisSpacing: 8,
          mainAxisSpacing: 8,
        ),
        itemBuilder: (_, i) => tiles[i],
      );
    });
  }
}

class _MetricTile extends StatelessWidget {
  final String label;
  final String value;
  const _MetricTile({required this.label, required this.value});

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    return AnimatedContainer(
      duration: const Duration(milliseconds: 150),
      curve: Curves.easeOut,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: cs.surfaceContainerHighest,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: cs.outlineVariant),
        boxShadow: [
          BoxShadow(
              blurRadius: 6,
              offset: const Offset(0, 1),
              color: cs.shadow.withValues(alpha: 0.05))
        ],
      ),
      child: Row(
        children: [
          Flexible(
            child: Text("$label:",
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: TextStyle(
                    fontWeight: FontWeight.w700, color: cs.onSurface)),
          ),
          const SizedBox(width: 6),
          Expanded(
            child: Text(value,
                textAlign: TextAlign.right,
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: TextStyle(color: cs.onSurfaceVariant)),
          ),
        ],
      ),
    );
  }
}

class _EstimateCard extends StatelessWidget {
  const _EstimateCard({required this.state});
  final DeviceState state;

  String _fmtDur(Duration d) {
    final days = d.inDays;
    final hrs = d.inHours % 24;
    final min = d.inMinutes % 60;
    final sec = d.inSeconds % 60;
    final dayPart = days > 0 ? "${days}d " : "";
    return "$dayPart${hrs.toString().padLeft(2, '0')}:${min.toString().padLeft(2, '0')}:${sec.toString().padLeft(2, '0')}";
  }

  String _bulanIndo(int m) {
    const nama = [
      "",
      "Januari",
      "Februari",
      "Maret",
      "April",
      "Mei",
      "Juni",
      "Juli",
      "Agustus",
      "September",
      "Oktober",
      "November",
      "Desember",
    ];
    if (m < 1 || m > 12) return m.toString();
    return nama[m];
  }

  String _fmtEta(DateTime now, Duration d) {
    final eta = now.add(d);
    final hh = eta.hour.toString().padLeft(2, '0');
    final mm = eta.minute.toString().padLeft(2, '0');
    final dd = eta.day.toString().padLeft(2, '0');
    final monthName = _bulanIndo(eta.month);
    final year = eta.year.toString();
    return "$dd-$monthName-$year, $hh:$mm";
  }

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    final est = state.estimate;

    IconData icon;
    String title;
    String bigTime;
    String sub;
    Color accent;
    String methodText = est?.etaMethodLabel ?? "";

    if (est == null) {
      icon = Icons.info_outline;
      title = "Estimasi baterai";
      bigTime = "Belum cukup data";
      sub = "Menunggu tren SoC yang stabil";
      accent = cs.outline;
      methodText = "";
    } else if (est.charging && est.toFull != null) {
      icon = Icons.battery_charging_full;
      title = "Mengisi Baterai";
      bigTime = _fmtDur(est.toFull!);
      sub = "Penuh => ${_fmtEta(est.calcTime, est.toFull!)}";
      accent = Colors.green;
    } else if (!est.charging && est.toEmpty != null) {
      icon = Icons.timer;
      title = "Estimasi Baterai";
      bigTime = _fmtDur(est.toEmpty!);
      sub = "Habis => ${_fmtEta(est.calcTime, est.toEmpty!)}";
      accent = Colors.orange;
    } else {
      icon = Icons.insights_outlined;
      title = "Estimasi (arus sangat kecil)";
      bigTime = "—";
      sub = "Menunggu Arus Berubah Signifikan";
      accent = cs.outline;
      methodText = "";
    }

    return Container(
      margin: const EdgeInsets.only(top: 6, bottom: 2),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
      decoration: BoxDecoration(
        color: cs.surfaceContainerHighest,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: cs.outlineVariant),
        boxShadow: [
          BoxShadow(
              blurRadius: 6,
              offset: const Offset(0, 1),
              color: cs.shadow.withValues(alpha: 0.05))
        ],
      ),
      child: Row(
        children: [
          Container(
            width: 38,
            height: 38,
            decoration: BoxDecoration(
              color: accent.withValues(alpha: 0.12),
              borderRadius: BorderRadius.circular(10),
            ),
            child: Icon(icon, color: accent),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(title,
                    style: TextStyle(
                      fontWeight: FontWeight.w700,
                      color: cs.onSurface,
                    )),
                const SizedBox(height: 4),
                Text(
                  bigTime,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: TextStyle(
                    fontSize: 20,
                    fontWeight: FontWeight.w800,
                    color: cs.onSurface,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  sub,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: TextStyle(color: cs.onSurfaceVariant),
                ),
                if (methodText.isNotEmpty) ...[
                  const SizedBox(height: 2),
                  Text(
                    methodText,
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    style: Theme.of(context).textTheme.bodySmall?.copyWith(
                          color: cs.onSurfaceVariant,
                          fontStyle: FontStyle.italic,
                        ),
                  ),
                ],
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _ChartPane extends StatefulWidget {
  const _ChartPane({required this.state, required this.controller});
  final DeviceState state;
  final TabController controller;

  @override
  State<_ChartPane> createState() => _ChartPaneState();
}

class _ChartPaneState extends State<_ChartPane> {
  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        TabBar(
          isScrollable: false,
          controller: widget.controller,
          tabs: const [
            Tab(text: "Tegangan"),
            Tab(text: "Arus"),
            Tab(text: "Suhu"),
            Tab(text: "ETA"),
          ],
        ),
        Expanded(
          child: TabBarView(
            controller: widget.controller,
            physics: const NeverScrollableScrollPhysics(),
            children: [
              _ValueChart(
                title: "Tegangan (V)",
                color: Colors.blue,
                selector: (r) => r.voltx,
                state: widget.state,
              ),
              _ValueChart(
                title: "Arus (A)",
                color: Colors.cyan,
                selector: (r) => r.ampx,
                state: widget.state,
              ),
              _TempChart(state: widget.state),
              _EstimateChart(state: widget.state),
            ],
          ),
        ),
      ],
    );
  }
}

// ============================================================
// Value / Temp chart (line) – sama seperti sebelumnya
// ============================================================

class _ValueChart extends StatefulWidget {
  const _ValueChart({
    required this.title,
    required this.color,
    required this.selector,
    required this.state,
  });

  final String title;
  final Color color;
  final double Function(ParsedReading) selector;
  final DeviceState state;

  @override
  State<_ValueChart> createState() => _ValueChartState();
}

class _ValueChartState extends State<_ValueChart> {
  // window 2 menit (live + scroll)
  static const Duration _viewWindow = Duration(minutes: 2);
  double? _viewStartX; // null = live

  @override
  Widget build(BuildContext context) {
    final items = widget.state.history.items;

    final data = items
        .map((r) =>
            FlSpot(r.ts.millisecondsSinceEpoch.toDouble(), widget.selector(r)))
        .toList();

    if (data.isEmpty) {
      return const Center(child: Text("Tidak ada data"));
    }

    final xMin = data.first.x;
    final xMax = data.last.x;
    final totalDuration = xMax - xMin;
    final viewWindowMs = _viewWindow.inMilliseconds.toDouble();

    double visibleStartX;
    double visibleEndX;

    if (totalDuration <= viewWindowMs) {
      visibleStartX = xMin;
      visibleEndX = xMax;
      _viewStartX = null;
    } else if (_viewStartX == null) {
      visibleEndX = xMax;
      visibleStartX = xMax - viewWindowMs;
    } else {
      final maxStartX = xMax - viewWindowMs;
      visibleStartX = _viewStartX!.clamp(xMin, maxStartX);
      visibleEndX = visibleStartX + viewWindowMs;
    }

    final visibleData = data
        .where((spot) => spot.x >= visibleStartX && spot.x <= visibleEndX)
        .toList();

    if (visibleData.isEmpty) {
      return const Center(child: Text("Tidak ada data dalam rentang ini"));
    }

    final yVals = visibleData.map((e) => e.y).toList();
    final yMin = yVals.reduce(min);
    final yMax = yVals.reduce(max);
    final yRange = (yMax - yMin).abs();
    final padding = max(yRange * 0.20, 0.1);
    final chartYMin = yMin - padding;
    final chartYMax = yMax + padding;

    final canScroll = totalDuration > viewWindowMs;
    final isLive = _viewStartX == null || !canScroll;

    return GestureDetector(
      onHorizontalDragUpdate: canScroll
          ? (details) {
              setState(() {
                final pixelsPerMs =
                    MediaQuery.of(context).size.width / viewWindowMs;
                final deltaMs = -details.delta.dx / pixelsPerMs;
                final maxStartX = xMax - viewWindowMs;

                if (_viewStartX == null) {
                  _viewStartX =
                      (visibleStartX + deltaMs).clamp(xMin, maxStartX);
                } else {
                  _viewStartX = (_viewStartX! + deltaMs).clamp(xMin, maxStartX);
                }
              });
            }
          : null,
      onHorizontalDragEnd: canScroll ? (_) {} : null,
      child: Padding(
        padding: const EdgeInsets.only(top: 6.0, right: 12, bottom: 8),
        child: Stack(
          children: [
            LineChart(
              LineChartData(
                minX: visibleStartX,
                maxX: visibleEndX,
                minY: chartYMin,
                maxY: chartYMax,
                gridData: const FlGridData(show: true),
                titlesData: FlTitlesData(
                  leftTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: true, reservedSize: 44),
                  ),
                  bottomTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 26,
                      interval:
                          const Duration(minutes: 1).inMilliseconds.toDouble(),
                      getTitlesWidget: (val, meta) {
                        final dt =
                            DateTime.fromMillisecondsSinceEpoch(val.toInt());
                        final hh = dt.hour.toString().padLeft(2, '0');
                        final mm = dt.minute.toString().padLeft(2, '0');
                        return Text("$hh:$mm",
                            style: Theme.of(context).textTheme.bodySmall);
                      },
                    ),
                  ),
                  rightTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false)),
                  topTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false)),
                ),
                lineTouchData: LineTouchData(
                  enabled: true,
                  touchTooltipData: LineTouchTooltipData(
                    getTooltipItems: (spots) {
                      return spots.map((spot) {
                        final dt =
                            DateTime.fromMillisecondsSinceEpoch(spot.x.toInt());
                        final hh = dt.hour.toString().padLeft(2, '0');
                        final mm = dt.minute.toString().padLeft(2, '0');
                        final ss = dt.second.toString().padLeft(2, '0');
                        return LineTooltipItem(
                          "$hh:$mm:$ss\n${spot.y.toStringAsFixed(2)}",
                          const TextStyle(color: Colors.white),
                        );
                      }).toList();
                    },
                  ),
                ),
                borderData: FlBorderData(show: true),
                lineBarsData: [
                  LineChartBarData(
                    isCurved: true,
                    spots: data,
                    barWidth: 1.6,
                    dotData: const FlDotData(show: false),
                    color: widget.color,
                  )
                ],
              ),
            ),
            if (totalDuration > viewWindowMs)
              Positioned(
                bottom: 0,
                left: 0,
                right: 0,
                child: Container(
                  height: 3,
                  margin: const EdgeInsets.symmetric(horizontal: 44),
                  child: CustomPaint(
                    painter: _ScrollIndicatorPainter(
                      totalRange: totalDuration,
                      visibleStart: visibleStartX - xMin,
                      visibleEnd: visibleEndX - xMin,
                      isAtEnd: isLive,
                    ),
                  ),
                ),
              ),
            Positioned(
              top: 0,
              right: 0,
              child: TextButton(
                style: TextButton.styleFrom(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                  minimumSize: const Size(0, 0),
                  tapTargetSize: MaterialTapTargetSize.shrinkWrap,
                ),
                onPressed: canScroll
                    ? () {
                        setState(() {
                          _viewStartX = null;
                        });
                      }
                    : null,
                child: Text(
                  "LIVE",
                  style: TextStyle(
                    fontSize: 10,
                    fontWeight: FontWeight.bold,
                    color: isLive
                        ? Colors.teal
                        : Theme.of(context).colorScheme.onSurfaceVariant,
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _ScrollIndicatorPainter extends CustomPainter {
  final double totalRange;
  final double visibleStart;
  final double visibleEnd;
  final bool isAtEnd;

  _ScrollIndicatorPainter({
    required this.totalRange,
    required this.visibleStart,
    required this.visibleEnd,
    required this.isAtEnd,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (totalRange <= 0) return;

    final paint = Paint()
      ..color = Colors.grey.withOpacity(0.3)
      ..style = PaintingStyle.fill;

    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), paint);

    final startFraction = visibleStart / totalRange;
    final endFraction = visibleEnd / totalRange;

    final indicatorPaint = Paint()
      ..color = isAtEnd ? Colors.teal : Colors.blue
      ..style = PaintingStyle.fill;

    final indicatorStart = startFraction * size.width;
    final indicatorWidth = (endFraction - startFraction) * size.width;

    canvas.drawRect(
      Rect.fromLTWH(indicatorStart, 0, indicatorWidth, size.height),
      indicatorPaint,
    );
  }

  @override
  bool shouldRepaint(_ScrollIndicatorPainter oldDelegate) =>
      oldDelegate.visibleStart != visibleStart ||
      oldDelegate.visibleEnd != visibleEnd ||
      oldDelegate.isAtEnd != isAtEnd;
}

class _TempChart extends StatefulWidget {
  const _TempChart({required this.state});
  final DeviceState state;

  @override
  State<_TempChart> createState() => _TempChartState();
}

class _TempChartState extends State<_TempChart> {
  static const Duration _viewWindow = Duration(minutes: 2);
  double? _viewStartX;

  @override
  Widget build(BuildContext context) {
    final items = widget.state.history.items;

    if (items.isEmpty) {
      return const Center(child: Text("Tidak ada data"));
    }

    final g = <FlSpot>[];
    final y = <FlSpot>[];
    final r = <FlSpot>[];

    for (final m in items) {
      final x = m.ts.millisecondsSinceEpoch.toDouble();
      if (m.tempx < 30) {
        g.add(FlSpot(x, m.tempx));
      } else if (m.tempx <= 50) {
        y.add(FlSpot(x, m.tempx));
      } else {
        r.add(FlSpot(x, m.tempx));
      }
    }

    final allSpots = [...g, ...y, ...r];
    if (allSpots.isEmpty) {
      return const Center(child: Text("Tidak ada data"));
    }

    final xMin = allSpots.map((e) => e.x).reduce(min);
    final xMax = allSpots.map((e) => e.x).reduce(max);
    final totalDuration = xMax - xMin;
    final viewWindowMs = _viewWindow.inMilliseconds.toDouble();

    double visibleStartX;
    double visibleEndX;

    if (totalDuration <= viewWindowMs) {
      visibleStartX = xMin;
      visibleEndX = xMax;
      _viewStartX = null;
    } else if (_viewStartX == null) {
      visibleEndX = xMax;
      visibleStartX = xMax - viewWindowMs;
    } else {
      final maxStartX = xMax - viewWindowMs;
      visibleStartX = _viewStartX!.clamp(xMin, maxStartX);
      visibleEndX = visibleStartX + viewWindowMs;
    }

    final visibleSpots = allSpots
        .where((spot) => spot.x >= visibleStartX && spot.x <= visibleEndX)
        .toList();

    if (visibleSpots.isEmpty) {
      return const Center(child: Text("Tidak ada data dalam rentang ini"));
    }

    final yVals = visibleSpots.map((e) => e.y).toList();
    final yMin = yVals.reduce(min);
    final yMax = yVals.reduce(max);
    final padding = max((yMax - yMin) * 0.15, 1.0);

    final canScroll = totalDuration > viewWindowMs;
    final isLive = _viewStartX == null || !canScroll;

    return GestureDetector(
      onHorizontalDragUpdate: canScroll
          ? (details) {
              setState(() {
                final pixelsPerMs =
                    MediaQuery.of(context).size.width / viewWindowMs;
                final deltaMs = -details.delta.dx / pixelsPerMs;
                final maxStartX = xMax - viewWindowMs;

                if (_viewStartX == null) {
                  _viewStartX =
                      (visibleStartX + deltaMs).clamp(xMin, maxStartX);
                } else {
                  _viewStartX = (_viewStartX! + deltaMs).clamp(xMin, maxStartX);
                }
              });
            }
          : null,
      onHorizontalDragEnd: canScroll ? (_) {} : null,
      child: Padding(
        padding: const EdgeInsets.only(top: 6.0, right: 12, bottom: 8),
        child: Stack(
          children: [
            LineChart(
              LineChartData(
                minX: visibleStartX,
                maxX: visibleEndX,
                minY: yMin - padding,
                maxY: yMax + padding,
                gridData: const FlGridData(show: true),
                titlesData: FlTitlesData(
                  leftTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: true, reservedSize: 44),
                  ),
                  bottomTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 26,
                      interval:
                          const Duration(minutes: 1).inMilliseconds.toDouble(),
                      getTitlesWidget: (val, meta) {
                        final dt =
                            DateTime.fromMillisecondsSinceEpoch(val.toInt());
                        final hh = dt.hour.toString().padLeft(2, '0');
                        final mm = dt.minute.toString().padLeft(2, '0');
                        return Text("$hh:$mm",
                            style: Theme.of(context).textTheme.bodySmall);
                      },
                    ),
                  ),
                  rightTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false)),
                  topTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false)),
                ),
                lineTouchData: LineTouchData(
                  enabled: true,
                  touchTooltipData: LineTouchTooltipData(
                    getTooltipItems: (spots) {
                      return spots.map((spot) {
                        final dt =
                            DateTime.fromMillisecondsSinceEpoch(spot.x.toInt());
                        final hh = dt.hour.toString().padLeft(2, '0');
                        final mm = dt.minute.toString().padLeft(2, '0');
                        final ss = dt.second.toString().padLeft(2, '0');
                        return LineTooltipItem(
                          "$hh:$mm:$ss\n${spot.y.toStringAsFixed(1)}°C",
                          const TextStyle(color: Colors.white),
                        );
                      }).toList();
                    },
                  ),
                ),
                borderData: FlBorderData(show: true),
                lineBarsData: [
                  if (g.isNotEmpty)
                    LineChartBarData(
                      isCurved: true,
                      spots: g,
                      barWidth: 1.4,
                      dotData: const FlDotData(show: false),
                      color: Colors.green,
                    ),
                  if (y.isNotEmpty)
                    LineChartBarData(
                      isCurved: true,
                      spots: y,
                      barWidth: 1.5,
                      dotData: const FlDotData(show: false),
                      color: Colors.yellowAccent,
                    ),
                  if (r.isNotEmpty)
                    LineChartBarData(
                      isCurved: true,
                      spots: r,
                      barWidth: 1.6,
                      dotData: const FlDotData(show: false),
                      color: Colors.redAccent,
                    ),
                ],
              ),
            ),
            if (totalDuration > viewWindowMs)
              Positioned(
                bottom: 0,
                left: 0,
                right: 0,
                child: Container(
                  height: 3,
                  margin: const EdgeInsets.symmetric(horizontal: 44),
                  child: CustomPaint(
                    painter: _ScrollIndicatorPainter(
                      totalRange: totalDuration,
                      visibleStart: visibleStartX - xMin,
                      visibleEnd: visibleEndX - xMin,
                      isAtEnd: isLive,
                    ),
                  ),
                ),
              ),
            Positioned(
              top: 0,
              right: 0,
              child: TextButton(
                style: TextButton.styleFrom(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                  minimumSize: const Size(0, 0),
                  tapTargetSize: MaterialTapTargetSize.shrinkWrap,
                ),
                onPressed: canScroll
                    ? () {
                        setState(() {
                          _viewStartX = null;
                        });
                      }
                    : null,
                child: Text(
                  "LIVE",
                  style: TextStyle(
                    fontSize: 10,
                    fontWeight: FontWeight.bold,
                    color: isLive
                        ? Colors.teal
                        : Theme.of(context).colorScheme.onSurfaceVariant,
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// ============================================================
// ETA BAR CHART ala battery graph HP
// ============================================================

class _EstimateChart extends StatelessWidget {
  const _EstimateChart({required this.state});
  final DeviceState state;

  double _interpSocFromHist(
      List<MapEntry<DateTime, double>> hist, DateTime t, double fallback) {
    if (hist.isEmpty) return fallback;
    if (t.isBefore(hist.first.key)) return hist.first.value;
    if (t.isAfter(hist.last.key)) return hist.last.value;

    for (int i = 0; i < hist.length - 1; i++) {
      final a = hist[i];
      final b = hist[i + 1];
      if (!t.isBefore(a.key) && !t.isAfter(b.key)) {
        final totalMs =
            (b.key.millisecondsSinceEpoch - a.key.millisecondsSinceEpoch)
                .toDouble();
        if (totalMs <= 0) return a.value;
        final frac =
            (t.millisecondsSinceEpoch - a.key.millisecondsSinceEpoch) / totalMs;
        return a.value + (b.value - a.value) * frac;
      }
    }
    return hist.last.value;
  }

  // cari arus “terdekat” di sekitar waktu t
  double _currentAt(DeviceState state, DateTime t) {
    final readings = state.history.items;
    if (readings.isEmpty) return 0.0;

    ParsedReading closest = readings.first;
    int bestDiff = (t.difference(closest.ts).inMilliseconds).abs();

    for (final r in readings) {
      final diff = (t.difference(r.ts).inMilliseconds).abs();
      if (diff < bestDiff) {
        bestDiff = diff;
        closest = r;
      }
    }
    return closest.ampx;
  }

  String _formatBottomLabel(DateTime time, Duration span) {
    if (span.inDays >= 2) {
      final d = time.day.toString().padLeft(2, '0');
      final m = time.month.toString().padLeft(2, '0');
      return "$d/$m";
    } else {
      final h = time.hour.toString().padLeft(2, '0');
      final m = time.minute.toString().padLeft(2, '0');
      return "$h:$m";
    }
  }

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    final est = state.estimate;
    final hist = state.socHist;

    if (est == null && hist.isEmpty) {
      return const Center(child: Text("Belum ada estimasi"));
    }

    final now = est?.calcTime ?? DateTime.now();

    // start: earliest SoC history (maks 15 menit ke belakang)
    final pastStart = hist.isNotEmpty
        ? hist.first.key
        : now.subtract(const Duration(minutes: 15));

    // apakah ETA valid?
    final bool hasEta;
    Duration etaDur;

    if (est != null) {
      if (est.charging && est.toFull != null && !est.toFull!.isNegative) {
        hasEta = true;
        etaDur = est.toFull!;
      } else if (!est.charging &&
          est.toEmpty != null &&
          !est.toEmpty!.isNegative) {
        hasEta = true;
        etaDur = est.toEmpty!;
      } else {
        hasEta = false;
        etaDur = Duration.zero;
      }
    } else {
      hasEta = false;
      etaDur = Duration.zero;
    }

    // end: waktu ETA kalau ada; kalau tidak, mentok di "now" saja
    final futureEnd = hasEta ? now.add(etaDur) : now;
    final totalSpan = futureEnd.difference(pastStart);

    if (totalSpan.isNegative || totalSpan.inSeconds == 0) {
      return const Center(child: Text("Rentang waktu estimasi tidak valid"));
    }

    const int bucketCount = 36;
    final totalMs = totalSpan.inMilliseconds.toDouble();
    final bucketMs = totalMs / bucketCount;

    final groups = <BarChartGroupData>[];

    for (int i = 0; i < bucketCount; i++) {
      final centerMs =
          pastStart.millisecondsSinceEpoch + ((i + 0.5) * bucketMs).round();
      final centerTime = DateTime.fromMillisecondsSinceEpoch(centerMs);

      final bool isFuture = hasEta && centerTime.isAfter(now);

      double socVal;
      Color color;

      if (!isFuture) {
        // BAR HISTORY – pakai SoC history, warna dari arus saat itu
        final halfBucket = Duration(milliseconds: (bucketMs / 2).round());
        final t0 = centerTime.subtract(halfBucket);
        final t1 = centerTime.add(halfBucket);
        final s0 = _interpSocFromHist(hist, t0, est?.socNow ?? 50.0);
        final s1 = _interpSocFromHist(hist, t1, est?.socNow ?? 50.0);
        socVal = s1.clamp(0.0, 100.0);

        final I = _currentAt(state, centerTime);

        if (I > 0.002) {
          // charging
          color = Colors.greenAccent.shade400;
        } else if (I < -0.002) {
          // usage
          color = Colors.lightBlueAccent.shade400;
        } else {
          color = Colors.grey.withOpacity(0.35);
        }
      } else {
        // BAR ETA – hanya kalau ETA benar-benar ada
        if (!hasEta || etaDur.inSeconds == 0) continue;

        final elapsed = centerTime.difference(now).inMilliseconds.toDouble();
        final frac = (elapsed / etaDur.inMilliseconds).clamp(0.0, 1.0);

        final targetSoc = est!.charging ? 100.0 : 0.0;
        final startSoc = est.socNow;
        socVal = (startSoc + (targetSoc - startSoc) * frac).clamp(0.0, 100.0);

        color = est.charging
            ? Colors.teal.withOpacity(0.55)
            : Colors.grey.withOpacity(0.55);
      }

      groups.add(
        BarChartGroupData(
          x: i,
          barRods: [
            BarChartRodData(
              toY: socVal,
              fromY: 0,
              width: 6,
              borderRadius: const BorderRadius.vertical(
                top: Radius.circular(2),
              ),
              color: color,
            ),
          ],
        ),
      );
    }

    if (groups.isEmpty) {
      return const Center(child: Text("Belum ada data untuk ETA chart"));
    }

    final span = totalSpan;

    return Column(
      children: [
        Expanded(
          child: Padding(
            padding: const EdgeInsets.only(top: 65.0, right: 10, bottom: 2),
            child: BarChart(
              BarChartData(
                maxY: 100,
                minY: 0,
                barGroups: groups,
                gridData: const FlGridData(show: true),
                titlesData: FlTitlesData(
                  leftTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: true, reservedSize: 36),
                  ),
                  rightTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false)),
                  topTitles: const AxisTitles(
                      sideTitles: SideTitles(showTitles: false)),
                  bottomTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 26,
                      getTitlesWidget: (value, meta) {
                        final i = value.round();
                        if (i < 0 || i >= bucketCount) {
                          return const SizedBox.shrink();
                        }

                        DateTime timeForIndex(int idx) {
                          final ms = pastStart.millisecondsSinceEpoch +
                              ((idx + 0.5) * bucketMs).round();
                          return DateTime.fromMillisecondsSinceEpoch(ms);
                        }

                        String label;
                        if (i == 0) {
                          label = _formatBottomLabel(timeForIndex(0), span);
                        } else if (i == bucketCount ~/ 2 && hasEta) {
                          label = "";
                        } else if (i == bucketCount - 1 && hasEta) {
                          label = _formatBottomLabel(
                              timeForIndex(bucketCount - 1), span);
                        } else {
                          return const SizedBox.shrink();
                        }

                        return Text(
                          label,
                          style: Theme.of(context).textTheme.bodySmall,
                        );
                      },
                    ),
                  ),
                ),
                borderData: FlBorderData(show: true),
                barTouchData: BarTouchData(
                  enabled: true,
                  touchTooltipData: BarTouchTooltipData(
                    getTooltipItem: (group, groupIndex, rod, rodIndex) {
                      final idx = group.x;
                      final ms = pastStart.millisecondsSinceEpoch +
                          ((idx + 0.5) * bucketMs).round();
                      final t = DateTime.fromMillisecondsSinceEpoch(ms);

                      final dd = t.day.toString().padLeft(2, '0');
                      final mo = t.month.toString().padLeft(2, '0');
                      final yy = t.year.toString();
                      final hh = t.hour.toString().padLeft(2, '0');
                      final mm = t.minute.toString().padLeft(2, '0');
                      final ss = t.second.toString().padLeft(2, '0');

                      final tsLabel = "$dd/$mo/$yy $hh:$mm:$ss";

                      return BarTooltipItem(
                        "$tsLabel\n${rod.toY.toStringAsFixed(1)}%",
                        const TextStyle(color: Colors.white),
                      );
                    },
                  ),
                ),
              ),
            ),
          ),
        ),
        const SizedBox(height: 4),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _LegendDot(color: Colors.greenAccent.shade400),
            const SizedBox(width: 4),
            Text("Mengisi", style: TextStyle(color: cs.onSurfaceVariant)),
            const SizedBox(width: 12),
            _LegendDot(color: Colors.lightBlueAccent),
            const SizedBox(width: 4),
            Text("Penggunaan", style: TextStyle(color: cs.onSurfaceVariant)),
            const SizedBox(width: 12),
            _LegendDot(
                color: hasEta && est != null && est.charging
                    ? Colors.teal.withOpacity(0.55)
                    : Colors.grey.withOpacity(0.55)),
            const SizedBox(width: 4),
            Text("ETA", style: TextStyle(color: cs.onSurfaceVariant)),
          ],
        ),
        const SizedBox(height: 4),
      ],
    );
  }
}

class _LegendDot extends StatelessWidget {
  const _LegendDot({required this.color});
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 10,
      height: 10,
      decoration: BoxDecoration(color: color, shape: BoxShape.circle),
    );
  }
}
