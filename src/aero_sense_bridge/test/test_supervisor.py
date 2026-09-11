"""Windows the dashboard opens must not inherit what OpenCV did to the bridge's environment."""
import os

from aero_sense_bridge import supervisor


def test_spawned_processes_do_not_inherit_opencvs_qt_plugin_path(monkeypatch):
    """The bug: Gazebo and RViz aborted in Qt init because cv2 had pointed Qt at its plugins."""
    monkeypatch.setenv("QT_QPA_PLATFORM_PLUGIN_PATH", "/site-packages/cv2/qt/plugins")
    monkeypatch.setenv("QT_QPA_FONTDIR", "/site-packages/cv2/qt/fonts")
    monkeypatch.setenv("DISPLAY", ":0")

    env = supervisor._child_env()

    assert "QT_QPA_PLATFORM_PLUGIN_PATH" not in env
    assert "QT_QPA_FONTDIR" not in env
    assert env["DISPLAY"] == ":0"                 # the window still needs a screen
    assert os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]   # and the bridge's own env is untouched


def test_extra_variables_are_added_on_top():
    env = supervisor._child_env({"GZ_SIM_RESOURCE_PATH": "/models"})

    assert env["GZ_SIM_RESOURCE_PATH"] == "/models"


def test_an_unknown_viewer_is_refused_without_starting_anything():
    assert supervisor.open_viewer("blender") == {"opened": False, "reason": "unknown view 'blender'"}
