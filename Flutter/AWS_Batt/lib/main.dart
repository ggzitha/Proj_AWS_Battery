import 'dart:async';
import 'dart:math';
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

/// BLE UUIDs from your sketch — must be `final` (not const)
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
      themeMode: ThemeMode.system, // follow device setting
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
  final double v;
  final double a;
  final double t;
  double get watt => v * a;
  ParsedReading(this.ts, this.v, this.a, this.t);
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
  double _socPctEma = 0;
  DateTime? _lastSocTs;

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
  static const double _chargeAmpThreshold = 0.10; // >0.1A => charging

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
    // Debounce/cooldown guard
    final now = DateTime.now();
    if (now.isBefore(_nextAllowedScan)) {
      final waitMs = _nextAllowedScan.difference(now).inMilliseconds;
      _snack("Tunggu Cooldown untuk scan...");
      // schedule a single retry after cooldown if not already scanning
      _pendingRetryTimer?.cancel();
      _pendingRetryTimer = Timer(Duration(milliseconds: max(300, waitMs)), () {
        if (!_scanning) _startScan();
      });
      return;
    }

    if (_scanning) return;
    setState(() => _scanning = true);

    // Ensure previous streams are closed
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
        // Handle Android "Undocumented scan throttle" or any platform throttling
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
    // push next allowed time forward
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

  void _subscribe(DeviceState ds) {
    ds.notifySub?.cancel();

    final ch = QualifiedCharacteristic(
      deviceId: ds.device.id,
      serviceId: kServiceUuid,
      characteristicId: kCharUuid,
    );

    ds.notifySub = _ble.subscribeToCharacteristic(ch).listen((data) {
      try {
        final reading = _parsePayload(data);
        ds.last = reading;
        ds.history.add(reading);
        // update SoC EMA
        ds._socPctEma = _emaNext(
          ds._socPctEma,
          _socPercent(reading),
          reading.a > _chargeAmpThreshold
              ? _emaAlphaCharge
              : _emaAlphaDischarge,
        );
        ds._lastSocTs = reading.ts;
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

  double _emaNext(double prev, double next, double alpha) {
    if (prev == 0) return next;
    return prev + alpha * (next - prev);
  }

  /// SoC estimate:
  /// 1) IR-correct terminal voltage to approximate resting voltage.
  /// 2) Convert to per-cell voltage.
  /// 3) Piecewise linear mapping: Min -> Nom -> Max (0..50..100%).
  double _socPercent(ParsedReading r) {
    // I*R correction (mΩ -> Ω)
    final rPerCellOhm = (_rCellMilliOhm.clamp(0, 1000)) / 1000.0;
    final rSeries = rPerCellOhm * _seriesCells;
    final isCharging = r.a > _chargeAmpThreshold;
    final vRestPack =
    isCharging ? (r.v - r.a * rSeries) : (r.v + r.a.abs() * rSeries);

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
      // 0..50% between vmin..vnom
      pct = 50.0 * ((vCell - vmin) / (vnom - vmin));
    } else {
      // 50..100% between vnom..vmax
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

  ParsedReading _parsePayload(List<int> bytes) {
    final s = String.fromCharCodes(bytes).trim();
    double? t, a, v;
    for (final part in s.split(',')) {
      final kv = part.split('=');
      if (kv.length != 2) continue;
      final key = kv[0].trim();
      final val = double.tryParse(kv[1].trim());
      if (val == null) continue;
      if (key == 't') t = val;
      if (key == 'a') a = val;
      if (key == 'v') v = val;
    }
    if (t == null || a == null || v == null) {
      throw FormatException("Data tidak Valid '$s'");
    }
    return ParsedReading(DateTime.now(), v, a, t);
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
                      _snack("Nilai Resitansi Tidak Valid (0--1000 mΩ).");
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

  // Insets helper: keep above 3-button nav; respect gesture insets lightly
  EdgeInsets _screenInsets(BuildContext context) {
    final mq = MediaQuery.of(context);
    final bottomNav = mq.viewPadding.bottom; // non-zero in 3-button nav
    final gesture = mq.systemGestureInsets.bottom; // gesture areas
    final bottom =
    bottomNav > 0 ? bottomNav : min(12.0, gesture); // small cushion
    return EdgeInsets.only(
      left: max(mq.viewPadding.left, mq.systemGestureInsets.left),
      right: max(mq.viewPadding.right, mq.systemGestureInsets.right),
      bottom: bottom,
      top: mq.viewPadding.top, // status bar
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
    final contentHeightPortrait = fullHeight * 0.90;

    final chartPane = _ChartPane(state: state);
    final metricPane = _MetricPane(last: last);

    // Time slider placed between metrics and charts (portrait & landscape)
    final timeSlider = Padding(
      padding: const EdgeInsets.symmetric(vertical: 6.0),
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

    // Content (battery bar stays in title)
    final content = isPortrait
        ? ConstrainedBox(
      constraints: BoxConstraints(maxHeight: contentHeightPortrait),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          metricPane,            // metrics (2x2)
          timeSlider,            // slider moved here
          const SizedBox(height: 4),
          Flexible(child: chartPane), // charts (tabbed)
        ],
      ),
    )
        : SizedBox(
      height: 360,
      child: Row(
        children: [
          Expanded(
            flex: 3,
            child: Column(
              children: [
                metricPane,
                timeSlider,
                const SizedBox(height: 4),
                Expanded(child: chartPane),
              ],
            ),
          ),
          const SizedBox(width: 12),
          // If you want a second column for metrics in landscape,
          // you could add it here. For now we keep a single column.
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
              // battery bar remains in title always
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
                  color: Colors.red.withOpacity(.08),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Text(state.error!,
                    style: const TextStyle(color: Colors.redAccent)),
              ),

            // Controls (slider removed from here)
            Padding(
              padding: const EdgeInsets.only(bottom: 8.0),
              child: Wrap(
                spacing: 10,
                runSpacing: 8,
                crossAxisAlignment: WrapCrossAlignment.center,
                children: [
                  Tooltip(
                    message: "Putuskan Koneksi BLE",
                    child: ElevatedButton.icon(
                      onPressed: onDisconnect,
                      icon: const Icon(Icons.link_off),
                      label: const Text(" "),
                    ),
                  ),
                  Tooltip(
                    message: "Hubungkan Kembali BLE",
                    child: OutlinedButton.icon(
                      onPressed: onReconnect,
                      icon: const Icon(Icons.refresh),
                      label: const Text(" "),
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

/// Battery bar with stateful animation that moves from previous percent to next (no reset-to-0)
class _BatteryBar extends StatefulWidget {
  const _BatteryBar({
    super.key,
    required this.percent,
    this.compact = true, // we only use compact in title
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
  double _fraction = 0.0; // 0..1 current fill

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
    if ((target - _fraction).abs() < 0.001) return; // tiny change: skip
    _ac.stop();
    _anim = Tween<double>(begin: _fraction, end: target)
        .animate(CurvedAnimation(parent: _ac, curve: Curves.easeOutCubic))
      ..addListener(() {
        setState(() {
          _fraction = _anim.value;
        });
      });
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
    final trackColor = Colors.white;
    final textPct = (_fraction * 100.0).clamp(0.0, 100.0);
    final text = "${textPct.toStringAsFixed(0)}%";
    final fillColor = _colorFor(_fraction);

    final height = 18.0;
    final radius = 999.0;
    final fontSize = 11.0;
    final fontWeight = FontWeight.w700;

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
            widthFactor: _fraction,
            child: Container(color: fillColor),
          ),
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
      child: bar,
    );
  }
}

class _MetricPane extends StatelessWidget {
  const _MetricPane({required this.last});
  final ParsedReading? last;

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
      _MetricTile(label: "Voltage", value: "${r.v.toStringAsFixed(2)} V"),
      _MetricTile(label: "Current", value: "${r.a.toStringAsFixed(2)} A"),
      _MetricTile(label: "Temperature", value: "${r.t.toStringAsFixed(1)} °C"),
      _MetricTile(label: "Watt", value: "${(r.v * r.a).toStringAsFixed(2)} W"),
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
            color: cs.shadow.withOpacity(0.05),
          ),
        ],
      ),
      child: Row(
        children: [
          Flexible(
            child: Text(
              "$label:",
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style:
              TextStyle(fontWeight: FontWeight.w700, color: cs.onSurface),
            ),
          ),
          const SizedBox(width: 6),
          Expanded(
            child: Text(
              value,
              textAlign: TextAlign.right,
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: TextStyle(color: cs.onSurfaceVariant),
            ),
          ),
        ],
      ),
    );
  }
}

class _ChartPane extends StatelessWidget {
  const _ChartPane({required this.state});
  final DeviceState state;

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 3,
      child: Column(
        children: [
          const TabBar(
            isScrollable: false,
            tabs: [
              Tab(text: "Voltage"),
              Tab(text: "Current"),
              Tab(text: "Temp")
            ],
          ),
          Expanded(
            child: TabBarView(
              children: [
                _ValueChart(
                  title: "Voltage (V)",
                  color: Colors.blue,
                  selector: (r) => r.v,
                  state: state,
                ),
                _ValueChart(
                  title: "Current (A)",
                  color: Colors.cyan,
                  selector: (r) => r.a,
                  state: state,
                ),
                _TempChart(state: state),
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
              barWidth: 2.6,
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
    final maxOffset = _buffer - _window; // 8 minutes
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
      if (m.t < 30) {
        g.add(FlSpot(x, m.t));
      } else if (m.t <= 50) {
        y.add(FlSpot(x, m.t));
      } else {
        r.add(FlSpot(x, m.t));
      }
    }

    final yVals = windowItems.map((e) => e.t).toList();
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
              barWidth: 2.6,
              dotData: const FlDotData(show: false),
              color: Colors.green,
            ),
            LineChartBarData(
              isCurved: true,
              spots: y.isEmpty ? [] : y,
              barWidth: 2.6,
              dotData: const FlDotData(show: false),
              color: Colors.yellow.shade700,
            ),
            LineChartBarData(
              isCurved: true,
              spots: r.isEmpty ? [] : r,
              barWidth: 2.6,
              dotData: const FlDotData(show: false),
              color: Colors.redAccent,
            ),
          ],
        ),
      ),
    );
  }
}
