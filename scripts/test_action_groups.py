"""Action-group diagnostic script.

Uses the ``recovery`` framework to connect to KSP, print the current state of
the first few action groups, and toggle/set a chosen UI action group. This is
useful for verifying that action-group mappings are wired correctly before
using them in a flight script.

In this setup, kRPC action group numbers match the KSP UI labels 1-10
directly (empirically verified), so no conversion is needed. The script
accepts UI numbers and passes them straight through.

Examples::

    # Print states and toggle UI action group 3 (kRPC group 2)
    D:\\miniconda3\\envs\\KRPC\\python.exe scripts/test_action_groups.py --toggle 3

    # Explicitly set UI action group 2 to True
    D:\\miniconda3\\envs\\KRPC\\python.exe scripts/test_action_groups.py --group 2 --state on

    # Use a different vessel
    D:\\miniconda3\\envs\\KRPC\\python.exe scripts/test_action_groups.py \
        --vessel "Booster A" --toggle 3
"""

from __future__ import annotations

import argparse
import sys

from recovery import ConnectionManager

DEFAULT_VESSEL = "Booster 2"


def _ui_to_krpc(ui_group: int) -> int:
    """Return the kRPC action-group number for a KSP UI label.

    Empirically, this setup uses the same 1-10 numbering as the UI.
    """
    if not 1 <= ui_group <= 10:
        raise ValueError(f"UI action group must be 1-10, got {ui_group}")
    return ui_group


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read and toggle KSP action groups through the recovery framework."
    )
    parser.add_argument(
        "--vessel",
        default=DEFAULT_VESSEL,
        help=f"Vessel name to connect to (default: {DEFAULT_VESSEL}).",
    )
    parser.add_argument(
        "--toggle",
        type=int,
        metavar="N",
        help="Toggle UI action group N (1-10) and print the result.",
    )
    parser.add_argument(
        "--group",
        type=int,
        metavar="N",
        help="UI action group N (1-10) to set with --state.",
    )
    parser.add_argument(
        "--state",
        choices=["on", "off"],
        help="Explicitly set --group to on or off.",
    )
    args = parser.parse_args(argv)

    if args.state is not None and args.group is None:
        parser.error("--state requires --group")
    if args.group is not None and args.toggle is not None:
        parser.error("use either --toggle or --group, not both")

    with ConnectionManager(address="127.0.0.1") as km:
        b = km.add_booster("ag_test", args.vessel)
        b.start()
        ctrl = b.controls

        print(f"Connected to vessel: {args.vessel}")
        print("Current action group states (UI number -> kRPC index):")
        states_before = {}
        for ui_group in range(1, 11):
            krpc_group = _ui_to_krpc(ui_group)
            state = ctrl.get_action_group(krpc_group)
            states_before[ui_group] = state
            marker = "*" if ui_group in (2, 3) else " "
            print(f"  {marker} UI group {ui_group:2d} (kRPC {krpc_group:2d}): {state}")

        if args.toggle is not None:
            ui_group = args.toggle
            krpc_group = _ui_to_krpc(ui_group)
            before = states_before[ui_group]
            print(f"\nToggling UI action group {ui_group} (kRPC {krpc_group})...")
            ctrl.toggle_action_group(krpc_group)
            after = ctrl.get_action_group(krpc_group)
            print(f"  before: {before}")
            print(f"  after : {after}")

        elif args.group is not None:
            ui_group = args.group
            krpc_group = _ui_to_krpc(ui_group)
            target = args.state == "on"
            before = states_before[ui_group]
            print(f"\nSetting UI action group {ui_group} (kRPC {krpc_group}) to {args.state}...")
            ctrl.set_action_group(krpc_group, target)
            after = ctrl.get_action_group(krpc_group)
            print(f"  before: {before}")
            print(f"  after : {after}")

        else:
            print("\nNo --toggle or --group specified; only printed current states.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
