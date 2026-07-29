from pathlib import Path

from mission_reasoner import node


def test_default_system_description_file_uses_package_share_directory(monkeypatch):
    package_share = Path('/opt/ros/share/mission_reasoner')

    def get_package_share_directory(package_name):
        assert package_name == 'mission_reasoner'
        return str(package_share)

    monkeypatch.setattr(
        node,
        'get_package_share_directory',
        get_package_share_directory,
    )

    assert node.default_system_description_file() == str(
        package_share / 'config' / 'system_description.yaml'
    )
