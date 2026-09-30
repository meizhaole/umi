from glob import glob
from pathlib import Path
from setuptools import setup

package_name = "rebotarm_bringup"
config_files = [
    "config/driver_params.yaml",
    "config/rebotarm_hardware.yaml",
]


def _description_data_files(package_name):
    data_files = []
    source_root = Path(__file__).resolve().parent
    for model in ("RS", "DM"):
        model_root = source_root / "description" / model
        for path in sorted(model_root.rglob("*")):
            if not path.is_file():
                continue
            relative_dir = path.parent.relative_to(model_root)
            destination = f"share/{package_name}/description/{model}"
            if str(relative_dir) != ".":
                destination = f"{destination}/{relative_dir.as_posix()}"
            data_files.append(
                (destination, [path.relative_to(source_root).as_posix()])
            )
    return data_files


setup(
    name=package_name,
    version="0.3.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", config_files),
        *_description_data_files(package_name),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Launch, configuration, and description files for reBotArm.",
    license="Apache-2.0",
)
