from pathlib import Path
import shutil
import re
import logging
import argparse

from .porting_tools.package_xml_porter import PackageXMLPorter
from .porting_tools.cmake_lists_porter import CMakeListsPorter
from .porting_tools.launch_file_porter import LaunchFilePorter

KNOWN_PYTHON_EXTS = [
    ".py", ".pyc", ".py3", ".pyo", ".ipynb", ".pyw", ".pyd", ".pyi", "pyz",
    ".pzd", ".pywz", ".rpy"
]
KNOWN_CPP_EXTS = [
    ".C", ".c", ".c++", ".cc", ".cpp", ".h", ".hh", ".hpp"
]


class Migrator:
    def __init__(self, src: Path, dst: Path,
                 extra_rules_launch=[],
                 extra_rules_cmakelists=[],
                 extra_rules_packagexml=[]):
        self.src = src
        self.dst = dst
        # Final report
        self.reports = []
        # Files to find
        self.pkg_xml_path = None
        self.cmklists_path = None
        # Information
        self.files_to_transfer_by_hand = []
        self.blindly_transferred = []
        # Extra rules
        self.extra_rules_launch = extra_rules_launch
        self.extra_rules_cmakelists = extra_rules_cmakelists
        self.extra_rules_packagexml = extra_rules_packagexml

        src_path = Path(src).resolve()

        # Traverse the src directory to gather files and information
        warned_about_python = False
        warned_about_c = False
        self.num_files = 0
        for f_path in src_path.rglob("*"):
            if f_path.suffix in KNOWN_PYTHON_EXTS:
                if not warned_about_python:
                    logging.warning("Python files are omitted. Transfer \
                    by hand. (Are you sure this is a support package?)")
                    warned_about_python = True
                self.files_to_transfer_by_hand.append(str(f_path))
            elif f_path.suffix in KNOWN_CPP_EXTS:
                if not warned_about_c:
                    logging.warning("C/C++ files are omitted. Transfer \
                    by hand. (Are you sure this is a support package?)")
                    warned_about_c = True
                self.files_to_transfer_by_hand.append(str(f_path))
            elif f_path.name == "package.xml":
                self.pkg_xml_path = f_path
            elif f_path.name == "CMakeLists.txt":
                self.cmklists_path = f_path
            elif f_path.suffix in [".msg", ".srv"]:
                logging.error("Unexpected .msg/.srv files encountered. \
                Consider using a different porting tool as mentioned\
                in the readme.")
                self.files_to_transfer_by_hand.append(str(f_path))
            self.ros_aware_copy(
                f_path,
                dst.joinpath(f_path.relative_to(src_path))
            )

    def ros_aware_copy(self, src: Path, dst: Path):
        """ROS aware choice of copy function. Uses known data."""
        if src.is_dir():
            dst.mkdir(parents=True, exist_ok=True)
            return
        
        # File is known to need a specific conversion
        self.lookups = [
            (r".*\.launch$", self.copy_launch),
            (r"^CMakeLists.txt$", self.copy_cmakelists),
            (r"^package.xml$", self.copy_packagexml)
        ]
        for pattern, copy_function in self.lookups:
            if re.search(pattern, src.name):
                copy_function(src, dst)
                return
        # Unknown file format, just copy blindly
        logging.info(str(src) + " is copied without alteration.")
        self.blindly_transferred.append(str(src))
        shutil.copy2(src, dst)

    #########################
    #     COPY FUNCTIONS    #
    #########################
    def copy_launch(self, src, dst):
        conv = LaunchFilePorter(src)
        conv.port(dst, extra_rules=self.extra_rules_launch)
        self.reports.append(conv.generate_report())

    def copy_cmakelists(self, src, dst):
        conv = CMakeListsPorter(src)
        conv.port(dst, extra_rules=self.extra_rules_cmakelists)
        self.reports.append(conv.generate_report())

    def copy_packagexml(self, src, dst):
        conv = PackageXMLPorter(src)
        conv.port(dst, extra_rules=self.extra_rules_packagexml)
        self.reports.append(conv.generate_report())

    #########################
    #          HELPERS      #
    #########################
    def generate_report(self):
        rep = f"Package: {self.src.name}\n"
        if len(self.files_to_transfer_by_hand) > 0:
            rep += "\nFiles to transfer by hand:\n"
            rep += "\t" + "\n\t".join(self.files_to_transfer_by_hand)
        if len(self.blindly_transferred) > 0:
            rep += "\n\nFiles transferred without modification:\n"
            rep += "\t" + "\n\t".join(self.blindly_transferred)
        if len(self.reports) > 0:
            rep += "\n\nReports from the porting tools:\n"
            rep += "\n".join(filter(None, self.reports))
        rep += "\n"
        return rep


def main():
    # Set up arguments
    cliparser = argparse.ArgumentParser(
        description="Port a ROS-Industrial robot support \
            package from ROS1 to ROS2."
    )
    cliparser.add_argument(
        "ipath",
        help="Path to the ROS 1 package",
        type=str
    )
    cliparser.add_argument(
        "opath",
        help="Desired output path (new folder will be created there)",
        type=str
    )
    args = cliparser.parse_args()
    ipath = Path(args.ipath)
    opath = Path(args.opath)
    if not ipath.is_dir():
        raise FileNotFoundError("No input directory found, \
        ipath was: {}".format(args.ipath))
    if not opath.is_dir():
        raise FileNotFoundError("No output directory found, \
        opath was: {}".format(args.opath))
    mig = Migrator(ipath, opath)
    with open(opath / "porting_report.txt", "w") as report:
        report.write(mig.generate_report())


if __name__ == "__main__":
    main()