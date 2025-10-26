import 'dart:async';
import 'dart:math';
import 'dart:ui' show FontFeature; // needed for FontFeature.tabularFigures
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
  SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
    DeviceOrientation.portraitDown,
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);
  runApp(const MyApp());
}

/// BLE UUIDs — final (not const)
final Uuid kServiceUuid = Uuid.parse("91bad492-b950-4226-aa2b-4ede9fa42f59");
final Uuid kCharUuid = Uuid.parse("cba1d466-344c-4be3-ab3f-189daa0a16d8");

/// Target device names (no MAC hardcoding)
const targetNames = {
  "BATT-Mon_4",
  "BATT-Mon_5",
  "BATT-Mon_6",
};

/// Settings keys
const _kVmaxKey = "per_cell_vmax";
const _kVnomKey = "per_cell_vnom";
const _kVminKey = "per_cell_vmin";
const _kSeriesKey = "series_cells";
const _kRcellMilliOhmKey = "per_cell_r_milliohm";

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
  final double voltx; // Voltage
  final double ampx; // Current (positive = charging, negative = discharging)
  final double tempx; // Temperature
  double get watt => voltx * ampx;
  ParsedReading(this.ts, this.voltx, this.ampx, this.tempx);
}

/// Sliding window by time (keeps 10 minutes)
class TimeSeries {
  final Duration window; // e.g., 10 minutes
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
  final bool charging; // true if ampx > 0
  final double socNow; // 0..100
  final Duration? toEmpty; // if discharging
  final Duration? toFull; // if charging
  final DateTime calcTime;
  final List<FlSpot> futureSeries; // projected SoC vs time
  final List<FlSpot> pastSeries; // recent SoC history
  Estimate({
    required this.charging,
    required this.socNow,
    required this.toEmpty,
    required this.toFull,
    required this.calcTime,
    required this.futureSeries,
    required this.pastSeries,
  });
}

class DeviceState {
  final DiscoveredDevice device;
  StreamSubscription<ConnectionStateUpdate>? connSub;
  StreamSubscription<List<int>>? notifySub;
  final TimeSeries history = TimeSeries(window: const Duration(minutes: 10));
  ParsedReading? last;
  bool connected = false;
  String? error;
  bool expanded = true; // accordion open
  double windowOffset = 0.0; // 0..1 in 10m buffer (2m window)

  // SoC
  double _socPctEma = 0;
  DateTime? _lastSocTs;

  // SoC instantaneous history (cap to ~10min @ 2s ≈ 300)
  final List<MapEntry<DateTime, double>> socHist = [];
  int _lastEstimateLen = 0;
  Estimate? estimate;

  // RAW capture (for display without any rounding)
  String? lastRaw; // whole frame
  String? lastRawVoltx; // 'voltx=' (or legacy 'v=')
  String? lastRawAmpx; // 'ampx=' (or legacy 'a=')
  String? lastRawTempx; // 'tempx=' (or legacy 't=')

  DeviceState(this.device);

  void dispose() {
    connSub?.cancel();
    notifySub?.cancel();
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

  /// id -> DeviceState
  final Map<String, DeviceState> _devices = {};

  bool _scanning = false;

  // Scan throttle/debounce
  static const Duration _scanCooldown = Duration(seconds: 2);
  DateTime _nextAllowedScan = DateTime.fromMillisecondsSinceEpoch(0);
  Timer? _pendingRetryTimer;

  // ===== BMS Settings (defaults) =====
  double _vMax = 4.20; // per-cell Max Voltage
  double _vNom = 3.60; // per-cell Nominal Voltage
  double _vMin = 2.80; // per-cell Minimum Voltage
  int _seriesCells = 3; // default 3s
  double _rCellMilliOhm = 38; // default 38 mΩ @1kHz

  // SoC smoothing params
  static const double _emaAlphaCharge = 0.15; // smoother while charging
  static const double _emaAlphaDischarge = 0.25; // snappier when unplugged

  @override
  void initState() {
    super.initState();
    _loadSettings().then((_) => _initPermissionsThenScan());
  }

  @override
  void dispose() {
    _pendingRetryTimer?.cancel();
    _scanSub?.cancel();
    for (final d in _devices.values) {
      d.dispose();
    }
    super.dispose();
  }

  Future<void> _loadSettings() async {
    final sp = await SharedPreferences.getInstance();
    setState(() {
      _vMax = sp.getDouble(_kVmaxKey) ?? 4.20;
      _vNom = sp.getDouble(_kVnomKey) ?? 3.60;
      _vMin = sp.getDouble(_kVminKey) ?? 2.80;
      _seriesCells = (sp.getInt(_kSeriesKey) ?? 3).clamp(1, 10);
      _rCellMilliOhm = sp.getDouble(_kRcellMilliOhmKey) ?? 38.0;

      if (!(_vMin < _vNom && _vNom < _vMax)) {
        _vMax = 4.20;
        _vNom = 3.60;
        _vMin = 2.80;
      }
      if (_rCellMilliOhm < 0) _rCellMilliOhm = 38.0;
    });
  }

  Future<void> _saveSettings({
    required double vmax,
    required double vnom,
    required double vmin,
    required int series,
    required double rMilliOhm,
  }) async {
    if (!(vmin < vnom && vnom < vmax)) {
      _snack("Pengaturan Tegangan Error: Nilai Min < Nom < Max !!");
      return;
    }
    final sp = await SharedPreferences.getInstance();
    await sp.setDouble(_kVmaxKey, vmax);
    await sp.setDouble(_kVnomKey, vnom);
    await sp.setDouble(_kVminKey, vmin);
    await sp.setInt(_kSeriesKey, series);
    await sp.setDouble(_kRcellMilliOhmKey, rMilliOhm);
    setState(() {
      _vMax = vmax;
      _vNom = vnom;
      _vMin = vmin;
      _seriesCells = series;
      _rCellMilliOhm = rMilliOhm;
    });
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
    final now = DateTime.now();
    if (now.isBefore(_nextAllowedScan)) {
      final waitMs = _nextAllowedScan.difference(now).inMilliseconds;
      _snack("Tunggu Cooldown untuk scan...");
      _pendingRetryTimer?.cancel();
      _pendingRetryTimer = Timer(Duration(milliseconds: max(300, waitMs)), () {
        if (!_scanning) _startScan();
      });
      return;
    }

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
          _devices[d.id] = ds;
          setState(() {});
          _connectAndSubscribe(ds);
        }
      }, onError: (e) {
        _snack("Scan dibatasi, mohon tunggu sebentar...");
        _endScanWithCooldown();
      }, onDone: () {
        _endScanWithCooldown();
      });

      _nextAllowedScan = DateTime.now().add(_scanCooldown);
    } catch (e) {
      _snack("Scan error: $e");
      _endScanWithCooldown();
    }
  }

  void _endScanWithCooldown() {
    _scanSub?.cancel();
    _scanSub = null;
    setState(() => _scanning = false);
    _nextAllowedScan = DateTime.now().add(_scanCooldown);
  }

  void _stopScan() {
    _scanSub?.cancel();
    _scanSub = null;
    setState(() => _scanning = false);
    _nextAllowedScan = DateTime.now().add(_scanCooldown);
  }

  void _rescan() {
    _stopScan();
    _startScan();
  }

  Future<void> _connectAndSubscribe(DeviceState ds) async {
    ds.error = null;
    ds.connSub?.cancel();

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
          setState(() {});
          _subscribe(ds);
          break;
        case DeviceConnectionState.disconnected:
          ds.connected = false;
          setState(() {});
          Future.delayed(const Duration(seconds: 2), () {
            if (!mounted) return;
            if (ds.error != "Terputus") {
              _connectAndSubscribe(ds);
            }
          });
          break;
        default:
          break;
      }
    }, onError: (e) {
      ds.error = "Gagal Menghubungkan: $e";
      ds.connected = false;
      setState(() {});
    });
  }

  // ---------- NEW: token extraction & parsing helpers ----------

  /// Extract all k=v numeric tokens in one pass. Keeps the first seen value per key.
  /// Accepts "12." and normalizes it to "12.0".
  Map<String, String> _extractRawTokens(String raw) {
    final rx = RegExp(
      r'\b([a-zA-Z]+)\s*=\s*(-?\d+(?:\.\d*)?)\b', // allow optional digits after '.'
      caseSensitive: false,
    );
    final out = <String, String>{};
    for (final m in rx.allMatches(raw)) {
      final k = m.group(1)!.toLowerCase();
      var v = m.group(2)!.trim(); // exact numeric text like '12.' or '12.49'
      if (v.endsWith('.')) v = '${v}0'; // normalize '12.' -> '12.0'
      out.putIfAbsent(k, () => v); // keep first seen; don't overwrite
    }
    return out;
  }

  /// Priority parse from token map; never overwrites earlier (preferred) keys.
  ParsedReading _parsePayloadFromTokens(Map<String, String> toks) {
    double? _pickD(List<String> keys) {
      for (final k in keys) {
        final s = toks[k];
        if (s == null) continue;
        final d = double.tryParse(s);
        if (d != null) return d;
      }
      return null;
    }

    final voltx = _pickD(['voltx', 'voltage', 'v']);
    final ampx = _pickD(['ampx', 'ampere', 'a']);
    final tempx = _pickD(['tempx', 'temperature', 't']);

    if (tempx == null || ampx == null || voltx == null) {
      throw FormatException("Data tidak Valid (token hilang): $toks");
    }
    return ParsedReading(DateTime.now(), voltx, ampx, tempx);
  }

  // Back-compat wrapper (unused elsewhere, but handy if needed)
  ParsedReading _parsePayload(String s) =>
      _parsePayloadFromTokens(_extractRawTokens(s));

  // ------------------------------------------------------------

  void _subscribe(DeviceState ds) {
    ds.notifySub?.cancel();

    final ch = QualifiedCharacteristic(
      deviceId: ds.device.id,
      serviceId: kServiceUuid,
      characteristicId: kCharUuid,
    );

    ds.notifySub = _ble.subscribeToCharacteristic(ch).listen((data) {
      try {
        // RAW frame (sanitized but not modified)
        final raw = String.fromCharCodes(data)
            .replaceAll(RegExp(r'[\x00-\x1F]'), ' ')
            .trim();

        ds.lastRaw = raw;
        print(
            "RAW: $raw  |  rawV=${ds.lastRawVoltx} parsedV=${ds.last?.voltx}");

        // Single-pass tokenization
        final toks = _extractRawTokens(raw);

        // Keep exact text for display (prefer long keys)
        ds.lastRawVoltx = toks['voltx'] ?? toks['voltage'] ?? toks['v'];
        ds.lastRawAmpx = toks['ampx'] ?? toks['ampere'] ?? toks['a'];
        ds.lastRawTempx = toks['tempx'] ?? toks['temperature'] ?? toks['t'];

        // Numeric parsing (for charts/math) — no overwrite of priority
        final reading = _parsePayloadFromTokens(toks);
        ds.last = reading;
        ds.history.add(reading);

        // Charge/discharge logic - positive ampx = charging, negative = discharging
        final isCharging = reading.ampx > 0.0;

        // Update SoC — choose alpha by charging state
        final socInstant = _socPercent(reading);
        ds._socPctEma = _emaNext(
          ds._socPctEma,
          socInstant,
          (isCharging ? _emaAlphaCharge : _emaAlphaDischarge),
        );
        ds._lastSocTs = reading.ts;

        // Keep SoC history (cap ke 10 menit)
        ds.socHist.add(MapEntry(reading.ts, socInstant));
        final cutoff = DateTime.now().subtract(const Duration(minutes: 10));
        while (ds.socHist.isNotEmpty && ds.socHist.first.key.isBefore(cutoff)) {
          ds.socHist.removeAt(0);
        }

        // Recompute estimate tiap 5 data baru
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

  Estimate? _computeEstimate(DeviceState ds) {
    final n = ds.socHist.length;
    if (n < 5) return null;
    final slice = ds.socHist.sublist(n - 5, n);
    final t0 = slice.first.key;
    final tN = slice.last.key;
    final s0 = slice.first.value;
    final sN = slice.last.value;
    final dtSec = tN.difference(t0).inMilliseconds / 1000.0;
    if (dtSec <= 1) return null;

    final slope = (sN - s0) / dtSec; // % per second
    final isChargingBySign = (ds.last?.ampx ?? 0) > 0.0; // positive = charging

    Duration? toEmpty;
    Duration? toFull;

    if (isChargingBySign) {
      // Charging: slope should be positive
      if (slope > 1e-6) {
        final secondsToFull =
            ((100.0 - sN) / slope).clamp(0, 365 * 24 * 3600).toDouble();
        toFull = Duration(seconds: secondsToFull.round());
      } else {
        toFull = null;
      }
    } else {
      // Discharging: slope should be negative
      if (slope < -1e-6) {
        final secondsToEmpty =
            (sN / (-slope)).clamp(0, 365 * 24 * 3600).toDouble();
        toEmpty = Duration(seconds: secondsToEmpty.round());
      } else {
        toEmpty = null;
      }
    }

    // Past & future series
    final past = ds.socHist
        .map((e) => FlSpot(e.key.millisecondsSinceEpoch.toDouble(), e.value))
        .toList();

    final now = DateTime.now();
    final List<FlSpot> future = [];
    if ((isChargingBySign && slope > 1e-6) ||
        (!isChargingBySign && slope < -1e-6)) {
      final limitSec = 6 * 3600;
      final totalSec = isChargingBySign
          ? min(limitSec, toFull?.inSeconds ?? 0)
          : min(limitSec, toEmpty?.inSeconds ?? 0);
      for (int s = 0; s <= totalSec; s += 10) {
        final soc = (sN + slope * s).clamp(0.0, 100.0);
        future.add(FlSpot(
            now.add(Duration(seconds: s)).millisecondsSinceEpoch.toDouble(),
            soc));
      }
    }

    return Estimate(
      charging: isChargingBySign,
      socNow: sN.clamp(0.0, 100.0),
      toEmpty: toEmpty,
      toFull: toFull,
      calcTime: now,
      futureSeries: future,
      pastSeries: past,
    );
  }

  double _emaNext(double prev, double next, double alpha) {
    if (prev == 0) return next;
    return prev + alpha * (next - prev);
  }

  /// SoC estimate with I*R correction and Min/Nom/Max piecewise mapping
  double _socPercent(ParsedReading r) {
    // I*R correction (mΩ -> Ω)
    final rPerCellOhm = (_rCellMilliOhm.clamp(0, 1000)) / 1000.0;
    final rSeries = rPerCellOhm * _seriesCells;

    // Charging when ampx > 0, discharging when ampx < 0
    final isCharging = r.ampx > 0.0;
    final vRestPack = isCharging
        ? (r.voltx - r.ampx * rSeries) // Charging: subtract voltage drop
        : (r.voltx + r.ampx.abs() * rSeries); // Discharging: add voltage drop

    // Per-cell resting voltage
    final vCell = (vRestPack / _seriesCells).clamp(0.0, 1000.0);

    // Piecewise mapping using Min, Nom, Max
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

  Future<void> _disconnect(DeviceState ds) async {
    ds.error = "Terputus";
    await ds.notifySub?.cancel();
    await ds.connSub?.cancel();
    ds.connected = false;
    setState(() {});
  }

  Future<void> _reconnect(DeviceState ds) async {
    ds.error = null;
    await _disconnect(ds);
    await Future.delayed(const Duration(milliseconds: 200));
    _connectAndSubscribe(ds);
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
    int seriesTmp = _seriesCells;

    showModalBottomSheet(
      context: context,
      useSafeArea: true,
      showDragHandle: true,
      isScrollControlled: true,
      builder: (ctx) => DraggableScrollableSheet(
        expand: false,
        minChildSize: 0.35,
        initialChildSize: 0.60,
        maxChildSize: 0.90,
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
                  const SizedBox(width: 10),
                  Expanded(
                    child: TextField(
                      controller: vminCtl,
                      keyboardType:
                          const TextInputType.numberWithOptions(decimal: true),
                      decoration: const InputDecoration(
                        labelText: "Min-Voltage (V/cell)",
                        helperText: "Default 2.80",
                        prefixIcon: Icon(Icons.battery_alert),
                        border: OutlineInputBorder(),
                      ),
                    ),
                  ),
                  Expanded(
                    child: TextField(
                      controller: vnomCtl,
                      keyboardType:
                          const TextInputType.numberWithOptions(decimal: true),
                      decoration: const InputDecoration(
                        labelText: "Nom-Volt (V/cell)",
                        helperText: "Default 3.60",
                        prefixIcon: Icon(Icons.battery_charging_full),
                        border: OutlineInputBorder(),
                      ),
                    ),
                  ),
                  Expanded(
                    child: TextField(
                      controller: vmaxCtl,
                      keyboardType:
                          const TextInputType.numberWithOptions(decimal: true),
                      decoration: const InputDecoration(
                        labelText: "Max-Volt (V/cell)",
                        helperText: "Default 4.20",
                        prefixIcon: Icon(Icons.bolt),
                        border: OutlineInputBorder(),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 10),
              Row(
                children: [
                  const SizedBox(width: 10),
                  Expanded(
                    child: InputDecorator(
                      decoration: const InputDecoration(
                        labelText: "Series",
                        border: OutlineInputBorder(),
                        prefixIcon: Icon(Icons.grid_on),
                      ),
                      child: DropdownButtonHideUnderline(
                        child: DropdownButton<int>(
                          isExpanded: true,
                          value: seriesTmp,
                          onChanged: (v) {
                            if (v != null) {
                              seriesTmp = v;
                              (ctx as Element).markNeedsBuild();
                            }
                          },
                          items: [for (int i = 1; i <= 10; i++) i]
                              .map((e) => DropdownMenuItem(
                                  value: e, child: Text("${e}s")))
                              .toList(),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 10),
              Row(
                children: [
                  const SizedBox(width: 10),
                  Expanded(
                    child: TextField(
                      controller: rCellCtl,
                      keyboardType:
                          const TextInputType.numberWithOptions(decimal: true),
                      decoration: const InputDecoration(
                        labelText: "Resitansi Internal Baterai (mΩ)",
                        helperText: "Default 38 mΩ @ 1 kHz (0–1000)",
                        prefixIcon: Icon(Icons.speed),
                        border: OutlineInputBorder(),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              Align(
                alignment: Alignment.centerRight,
                child: FilledButton.icon(
                  onPressed: () {
                    final vmax = double.tryParse(vmaxCtl.text.trim());
                    final vnom = double.tryParse(vnomCtl.text.trim());
                    final vmin = double.tryParse(vminCtl.text.trim());
                    final rMilli = double.tryParse(rCellCtl.text.trim());
                    if (vmax == null || vnom == null || vmin == null) {
                      _snack("Masukan nilai tegangan yang valid.");
                      return;
                    }
                    if (!(vmin < vnom && vnom < vmax)) {
                      _snack("Nilai V harus : Min < Nom < Max.");
                      return;
                    }
                    if (rMilli == null || rMilli < 0 || rMilli > 1000) {
                      _snack("Nilai Resitansi Tidak Valid (0–1000 mΩ).");
                      return;
                    }
                    _saveSettings(
                      vmax: vmax,
                      vnom: vnom,
                      vmin: vmin,
                      series: seriesTmp,
                      rMilliOhm: rMilli,
                    );
                    Navigator.pop(ctx);
                  },
                  icon: const Icon(Icons.save),
                  label: const Text("Simpan"),
                ),
              ),
              const SizedBox(height: 12),
            ],
          ),
        ),
      ),
    );
  }

  // Insets helper
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
            message: "Pengaturan BMS (Min/Nom/Max V, Series, Internal R)",
            child: IconButton(
              onPressed: _openSettingsSheet,
              icon: const Icon(Icons.settings),
            ),
          ),
          Tooltip(
            message: "Scan Ulang Perangkat Batt",
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
                    onDisconnect: () => _disconnect(ds),
                    onReconnect: () => _reconnect(ds),
                    onWindowChanged: (v) => setState(() => ds.windowOffset = v),
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
    required this.onDisconnect,
    required this.onReconnect,
    required this.onWindowChanged,
  });

  final DeviceState state;
  final double socPercent;
  final VoidCallback onDisconnect;
  final VoidCallback onReconnect;
  final ValueChanged<double> onWindowChanged;

  @override
  Widget build(BuildContext context) {
    final last = state.last;
    final title =
        state.device.name.isNotEmpty ? state.device.name : state.device.id;

    final media = MediaQuery.of(context);
    final isPortrait = media.orientation == Orientation.portrait;
    final fullHeight = media.size.height;
    final contentHeightPortrait = fullHeight * 0.65;

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
      key: ValueKey("chart-${state.device.id}-${state.history.items.length}"),
      child: _ChartPane(state: state),
    );

    final timeSlider = Padding(
      padding: const EdgeInsets.symmetric(vertical: 1.0),
      child: ConstrainedBox(
        constraints: const BoxConstraints(minWidth: 160, maxWidth: 380),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Text("⇦"),
            Expanded(
              child: Slider(
                value: state.windowOffset,
                onChanged: onWindowChanged,
                divisions: 16,
                label: _windowLabel(state.windowOffset),
              ),
            ),
            const Text("⇨"),
          ],
        ),
      ),
    );

    final content = isPortrait
        ? ConstrainedBox(
            constraints: BoxConstraints(maxHeight: contentHeightPortrait),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                metricPane,
                estimateCard,
                timeSlider,
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
                      timeSlider,
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

            // Controls
            Padding(
              padding: const EdgeInsets.only(bottom: 2.0),
              child: Wrap(
                spacing: 30,
                runSpacing: 2,
                crossAxisAlignment: WrapCrossAlignment.center,
                children: [
                  Tooltip(
                    message: "Putuskan Koneksi BLE",
                    child: ElevatedButton.icon(
                      onPressed: onDisconnect,
                      icon: const Icon(Icons.link_off),
                      label: const Text(""),
                    ),
                  ),
                  Tooltip(
                    message: "Hubungkan Kembali BLE",
                    child: OutlinedButton.icon(
                      onPressed: onReconnect,
                      icon: const Icon(Icons.refresh),
                      label: const Text(""),
                    ),
                  ),
                ],
              ),
            ),

            content,
          ],
        ),
      ),
    );
  }

  String _windowLabel(double v) {
    final totalMs = const Duration(minutes: 8).inMilliseconds;
    final ms = (v * totalMs).round();
    final mins = (ms / 60000).floor();
    final secs = ((ms % 60000) / 1000).round();
    if (ms == 0) return "Now";
    return "-${mins}m${secs.toString().padLeft(2, '0')}s";
  }
}

/// Battery bar (title)
class _BatteryBar extends StatefulWidget {
  const _BatteryBar({
    super.key,
    required this.percent,
    this.compact = true,
  });

  final double percent; // 0..100 (smoothed)
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
    if (p < 25)
      return _lerpColor(
          const Color(0xFFE53935), const Color(0xFFFF7043), p / 25);
    if (p < 40)
      return _lerpColor(
          const Color(0xFFFF7043), const Color(0xFFFFEE58), (p - 25) / 15);
    if (p < 60)
      return _lerpColor(
          const Color(0xFFFFEE58), const Color(0xFF26C6DA), (p - 40) / 20);
    if (p < 75)
      return _lerpColor(
          const Color(0xFF26C6DA), const Color(0xFF66BB6A), (p - 60) / 15);
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

  // exact tokens coming from device (for display)
  final String? rawVoltx;
  final String? rawAmpx;
  final String? rawTempx;

  String _showRawOrDouble(String? raw, double? num, {int maxDecimals = 2}) {
    // accept raw only if like 12 or 12.49 (not "12.")
    final validRaw =
        (raw != null && RegExp(r'^-?\d+(?:\.\d+)?$').hasMatch(raw));
    if (validRaw) return raw!;
    if (num == null) return "—";
    String s = num.toStringAsFixed(maxDecimals);
    s = s.replaceFirst(
        RegExp(r'\.?0+$'), ''); // trim trailing zeros and trailing dot
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
          label: "Voltage",
          value: "${_showRawOrDouble(rawVoltx, r.voltx, maxDecimals: 2)} V"),
      _MetricTile(
          label: "Current",
          value: "${_showRawOrDouble(rawAmpx, r.ampx, maxDecimals: 2)} A"),
      _MetricTile(
          label: "Temperature",
          value: "${_showRawOrDouble(rawTempx, r.tempx, maxDecimals: 2)} °C"),
      _MetricTile(
          label: "Watt",
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

/// ===== Estimation Card =====
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

  String _fmtEta(DateTime now, Duration d) {
    final eta = now.add(d);
    final hh = eta.hour.toString().padLeft(2, '0');
    final mm = eta.minute.toString().padLeft(2, '0');
    final dd = eta.day.toString().padLeft(2, '0');
    final mon = eta.month.toString().padLeft(2, '0');
    return "$hh:$mm • $dd/$mon";
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

    if (est == null) {
      icon = Icons.info_outline;
      title = "Estimasi baterai";
      bigTime = "Belum cukup data";
      sub = "Menunggu 5 sampel terakhir";
      accent = cs.outline;
    } else if (est.charging && est.toFull != null) {
      icon = Icons.battery_charging_full;
      title = "Mengisi Baterai";
      bigTime = _fmtDur(est.toFull!);
      sub = "ETA ${_fmtEta(est.calcTime, est.toFull!)}";
      accent = Colors.green;
    } else if (!est.charging && est.toEmpty != null) {
      icon = Icons.timer;
      title = "Estimasi Baterai";
      bigTime = _fmtDur(est.toEmpty!);
      sub = "Habis sekitar ${_fmtEta(est.calcTime, est.toEmpty!)}";
      accent = Colors.orange;
    } else {
      icon = Icons.insights_outlined;
      title = "Estimasi tidak stabil";
      bigTime = "—";
      sub = "Perubahan arus/SoC terlalu kecil";
      accent = cs.outline;
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
                    fontFeatures: const [FontFeature.tabularFigures()],
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
              ],
            ),
          ),
        ],
      ),
    );
  }
}

/// ===== Charts (Voltage/Current/Temp/Est) =====
class _ChartPane extends StatelessWidget {
  const _ChartPane({required this.state});
  final DeviceState state;

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 4,
      child: Column(
        children: [
          const TabBar(
            isScrollable: false,
            tabs: [
              Tab(text: "Voltage"),
              Tab(text: "Current"),
              Tab(text: "Temp"),
              Tab(text: "Est"),
            ],
          ),
          Expanded(
            child: TabBarView(
              children: [
                _ValueChart(
                  title: "Voltage (V)",
                  color: Colors.blue,
                  selector: (r) => r.voltx,
                  state: state,
                ),
                _ValueChart(
                  title: "Current (A)",
                  color: Colors.cyan,
                  selector: (r) => r.ampx,
                  state: state,
                ),
                _TempChart(state: state),
                _EstimateChart(state: state),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _ValueChart extends StatelessWidget {
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

  static const _buffer = Duration(minutes: 10);
  static const _window = Duration(minutes: 2);

  @override
  Widget build(BuildContext context) {
    final items = state.history.items;
    final now = DateTime.now();
    final maxOffset = _buffer - _window; // 8 minutes
    final offsetMs = (state.windowOffset * maxOffset.inMilliseconds).round();
    final end = now.subtract(Duration(milliseconds: offsetMs));
    final start = end.subtract(_window);

    final data = items
        .where((r) => !r.ts.isBefore(start) && !r.ts.isAfter(end))
        .map((r) => FlSpot(r.ts.millisecondsSinceEpoch.toDouble(), selector(r)))
        .toList();

    final yVals = data.map((e) => e.y).toList();
    final yMin = yVals.isEmpty ? 0.0 : yVals.reduce(min);
    final yMax = yVals.isEmpty ? 1.0 : yVals.reduce(max);
    final pad = (yMax - yMin).abs() * 0.1 + 0.1;

    return Padding(
      padding: const EdgeInsets.only(top: 6.0),
      child: LineChart(
        LineChartData(
          minX: start.millisecondsSinceEpoch.toDouble(),
          maxX: end.millisecondsSinceEpoch.toDouble(),
          minY: yMin - pad,
          maxY: yMax + pad,
          gridData: const FlGridData(show: true),
          titlesData: FlTitlesData(
            leftTitles: const AxisTitles(
              sideTitles: SideTitles(showTitles: true, reservedSize: 38),
            ),
            bottomTitles: AxisTitles(
              sideTitles: SideTitles(
                showTitles: true,
                reservedSize: 26,
                getTitlesWidget: (val, meta) {
                  final dt = DateTime.fromMillisecondsSinceEpoch(val.toInt());
                  if (dt.second % 30 != 0) return const SizedBox.shrink();
                  final hh = dt.hour.toString().padLeft(2, '0');
                  final mm = dt.minute.toString().padLeft(2, '0');
                  final ss = dt.second.toString().padLeft(2, '0');
                  return Text("$hh:$mm:$ss",
                      style: Theme.of(context).textTheme.bodySmall);
                },
              ),
            ),
            rightTitles:
                const AxisTitles(sideTitles: SideTitles(showTitles: false)),
            topTitles:
                const AxisTitles(sideTitles: SideTitles(showTitles: false)),
          ),
          lineTouchData: const LineTouchData(enabled: true),
          borderData: FlBorderData(show: true),
          lineBarsData: [
            LineChartBarData(
              isCurved: true,
              spots: data.isEmpty
                  ? [FlSpot(end.millisecondsSinceEpoch.toDouble(), 0)]
                  : data,
              barWidth: 1.4,
              dotData: const FlDotData(show: false),
              color: color,
            )
          ],
        ),
      ),
    );
  }
}

class _TempChart extends StatelessWidget {
  const _TempChart({required this.state});
  final DeviceState state;

  static const _buffer = Duration(minutes: 10);
  static const _window = Duration(minutes: 2);

  @override
  Widget build(BuildContext context) {
    final items = state.history.items;
    final now = DateTime.now();
    final maxOffset = _buffer - _window;
    final offsetMs = (state.windowOffset * maxOffset.inMilliseconds).round();
    final end = now.subtract(Duration(milliseconds: offsetMs));
    final start = end.subtract(_window);

    final windowItems = items
        .where((r) => !r.ts.isBefore(start) && !r.ts.isAfter(end))
        .toList();

    final g = <FlSpot>[];
    final y = <FlSpot>[];
    final r = <FlSpot>[];

    for (final m in windowItems) {
      final x = m.ts.millisecondsSinceEpoch.toDouble();
      if (m.tempx < 30) {
        g.add(FlSpot(x, m.tempx));
      } else if (m.tempx <= 50) {
        y.add(FlSpot(x, m.tempx));
      } else {
        r.add(FlSpot(x, m.tempx));
      }
    }

    final yVals = windowItems.map((e) => e.tempx).toList();
    final yMin = yVals.isEmpty ? 0.0 : yVals.reduce(min);
    final yMax = yVals.isEmpty ? 1.0 : yVals.reduce(max);
    final pad = (yMax - yMin).abs() * 0.1 + 0.1;

    return Padding(
      padding: const EdgeInsets.only(top: 6.0),
      child: LineChart(
        LineChartData(
          minX: start.millisecondsSinceEpoch.toDouble(),
          maxX: end.millisecondsSinceEpoch.toDouble(),
          minY: yMin - pad,
          maxY: yMax + pad,
          gridData: const FlGridData(show: true),
          titlesData: FlTitlesData(
            leftTitles: const AxisTitles(
              sideTitles: SideTitles(showTitles: true, reservedSize: 38),
            ),
            bottomTitles: AxisTitles(
              sideTitles: SideTitles(
                showTitles: true,
                reservedSize: 26,
                getTitlesWidget: (val, meta) {
                  final dt = DateTime.fromMillisecondsSinceEpoch(val.toInt());
                  if (dt.second % 30 != 0) return const SizedBox.shrink();
                  final hh = dt.hour.toString().padLeft(2, '0');
                  final mm = dt.minute.toString().padLeft(2, '0');
                  final ss = dt.second.toString().padLeft(2, '0');
                  return Text("$hh:$mm:$ss",
                      style: Theme.of(context).textTheme.bodySmall);
                },
              ),
            ),
            rightTitles:
                const AxisTitles(sideTitles: SideTitles(showTitles: false)),
            topTitles:
                const AxisTitles(sideTitles: SideTitles(showTitles: false)),
          ),
          lineTouchData: const LineTouchData(enabled: true),
          borderData: FlBorderData(show: true),
          lineBarsData: [
            LineChartBarData(
              isCurved: true,
              spots: g.isEmpty ? [] : g,
              barWidth: 1.4,
              dotData: const FlDotData(show: false),
              color: Colors.green,
            ),
            LineChartBarData(
              isCurved: true,
              spots: y.isEmpty ? [] : y,
              barWidth: 1.5,
              dotData: const FlDotData(show: false),
              color: Colors.yellowAccent,
            ),
            LineChartBarData(
              isCurved: true,
              spots: r.isEmpty ? [] : r,
              barWidth: 1.6,
              dotData: const FlDotData(show: false),
              color: Colors.redAccent,
            ),
          ],
        ),
      ),
    );
  }
}

/// ===== Estimation Chart (history + projection) =====
class _EstimateChart extends StatelessWidget {
  const _EstimateChart({required this.state});
  final DeviceState state;

  @override
  Widget build(BuildContext context) {
    final est = state.estimate;
    final past = est?.pastSeries ?? [];
    final future = est?.futureSeries ?? [];

    final nowMs = DateTime.now().millisecondsSinceEpoch.toDouble();
    final minPastX = past.isEmpty
        ? nowMs - const Duration(minutes: 2).inMilliseconds
        : past.first.x;
    final maxFutureX = future.isEmpty
        ? nowMs + const Duration(minutes: 2).inMilliseconds
        : future.last.x;

    return Padding(
      padding: const EdgeInsets.only(top: 6.0),
      child: LineChart(
        LineChartData(
          minX: minPastX,
          maxX: maxFutureX,
          minY: 0,
          maxY: 100,
          gridData: const FlGridData(show: true),
          titlesData: FlTitlesData(
            leftTitles: const AxisTitles(
              sideTitles: SideTitles(showTitles: true, reservedSize: 36),
            ),
            bottomTitles: AxisTitles(
              sideTitles: SideTitles(
                showTitles: true,
                reservedSize: 26,
                getTitlesWidget: (val, meta) {
                  final dt = DateTime.fromMillisecondsSinceEpoch(val.toInt());
                  final hh = dt.hour.toString().padLeft(2, '0');
                  final mm = dt.minute.toString().padLeft(2, '0');
                  return Text("$hh:$mm",
                      style: Theme.of(context).textTheme.bodySmall);
                },
              ),
            ),
            rightTitles:
                const AxisTitles(sideTitles: SideTitles(showTitles: false)),
            topTitles:
                const AxisTitles(sideTitles: SideTitles(showTitles: false)),
          ),
          lineTouchData: const LineTouchData(enabled: true),
          borderData: FlBorderData(show: true),
          lineBarsData: [
            LineChartBarData(
              isCurved: true,
              spots: past.isEmpty ? [FlSpot(nowMs, 0)] : past,
              barWidth: 1.6,
              dotData: const FlDotData(show: false),
              color: Colors.teal,
            ),
            LineChartBarData(
              isCurved: true,
              spots: future,
              barWidth: 1.6,
              dotData: const FlDotData(show: false),
              color: Colors.grey,
            ),
          ],
        ),
      ),
    );
  }
}
