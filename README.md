# orcahand-inhouse

In-house tooling for driving the ORCA Touch hand. Start with **[HANDOFF.md](HANDOFF.md)** for current status.

- `servo-jog/` — PC-side raw servo jog page and diagnostics (Feetech STS over a USB adapter), plus `p4.py` / `watch.py` for the P4's JSON console.
- `firmware/vuehand_p4/` — ESP32-P4 hand firmware (from sitevue/playground.vuehand) with the Waveshare ESP32-P4-WIFI6 + Bus Servo Adapter (A) build; `firmware/waveshare-p4-changes.patch` is the diff against upstream.
- `hand-server/` — patch for the OrcaHand-Corndog-Pickup `/control` page and this hand's right-side calibration.
- `logs/` — calibration logs.
