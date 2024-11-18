# Copyright (c) 2024 Basalte bv
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import subprocess
import sys
import textwrap
from itertools import chain
from pathlib import Path

from west.commands import WestCommand
from zephyr_ext_common import ZEPHYR_BASE

sys.path.append(os.fspath(Path(__file__).parent.parent))
import zephyr_module


def in_venv() -> bool:
    return sys.prefix != sys.base_prefix


class Packages(WestCommand):
    def __init__(self):
        super().__init__(
            "packages",
            "manage packages for Zephyr",
            "List and Install packages for Zephyr and projects",
            accepts_unknown_args=True,
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=textwrap.dedent(
                """
            Listing packages:

                Run 'west packages <manager>' to list packages for Zephyr and active projects
                for a given package manager.
                See 'west packages <manager> --help' for details.
            """
            ),
        )

        parser.add_argument(
            "-p",
            "--project",
            action="append",
            default=[],
            dest="projects",
            help="Project to run the 'packages' command for. "
            "Use 'zephyr' if the 'packages' command should run for Zephyr itself. "
            "Option can be passed multiple times. "
            "If this option is not given, the 'packages' command will run for Zephyr "
            "and all active projects.",
        )

        subparsers_gen = parser.add_subparsers(
            metavar="<manager>",
            dest="manager",
            help="select a manager.",
            required=True,
        )

        pip_parser = subparsers_gen.add_parser(
            "pip",
            help="manage pip packages",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=textwrap.dedent(
                """
            Manage pip packages:

                Run 'west packages pip' to print all pip packages or requirements
                files needed by Zephyr and the active projects.

                The output is compatible with the requirements file format itself.
            """
            ),
        )

        pip_parser.add_argument(
            "--install",
            action="store_true",
            help="Install pip packages instead of listing them. "
            "A single 'pip install' command is built and executed. "
            "Additional pip arguments can be passed after a -- separator "
            "from the original 'west packages pip --install' command. For example pass "
            "'--dry-run' to not actually install anything, but print what would be.",
        )

        return parser

    def do_run(self, args, unknown):
        if len(unknown) > 0 and unknown[0] != "--":
            self.die(
                f'Unknown argument "{unknown[0]}"; '
                'arguments for the manager should be passed after "--"'
            )

        if args.manager == "pip":
            return self.do_run_pip(args, unknown[1:])

        # Unreachable but print an error message if an implementation is missing.
        self.die(f'Unsupported package manager: "{args.manager}"')

    def do_run_pip(self, args, manager_args):
        packages = []
        requirements = []

        if not args.projects or "zephyr" in args.projects:
            requirements.append(ZEPHYR_BASE / "scripts/requirements.txt")

        for module in zephyr_module.parse_modules(ZEPHYR_BASE, self.manifest):
            module_name = module.meta.get("name", None)
            if args.projects and module_name not in args.projects:
                continue

            pip = module.meta.get("package-managers", {}).get("pip")
            if pip is None:
                continue

            packages += pip.get("packages", [])
            requirements += [Path(module.project) / r for r in pip.get("requirements", [])]

        if args.install:
            if not in_venv():
                self.wrn("Running pip install outside of a virtual environment")

            subprocess.check_call(
                [sys.executable, "-m", "pip", "install"]
                + packages
                + list(chain.from_iterable([("-r", r) for r in requirements]))
                + manager_args
            )
            return

        if len(manager_args) > 0:
            self.die(f'west packages pip does not support unknown arguments: "{manager_args}"')

        self.inf("\n".join(packages + [f"-r {r}" for r in requirements]))
