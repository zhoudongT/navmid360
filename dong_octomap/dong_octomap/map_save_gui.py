#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
from pathlib import Path

import rclpy
from dong_map_msgs.srv import LoadNavigationMapPackage, SaveNavigationMapPackage

from dong_octomap.ros_client_timeouts import (
    load_navigation_map_client_spin_timeout_sec,
    save_navigation_map_client_spin_timeout_sec,
)

from PyQt5.QtCore import QThread, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor


class SaveMapClient(Node):
    def __init__(self) -> None:
        super().__init__(f"map_save_gui_save_client_{time.time_ns()}")
        self.client = self.create_client(
            SaveNavigationMapPackage, "/map_package_manager/save_package"
        )

    def save_package(
        self,
        package_path: str,
        overwrite: bool,
        fast_save: bool = False,
        timeout_sec: float | None = None,
    ):
        if timeout_sec is None:
            timeout_sec = save_navigation_map_client_spin_timeout_sec()
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, "保存服务 /map_package_manager/save_package 不可用。", ""

        request = SaveNavigationMapPackage.Request()
        request.package_path = package_path
        request.overwrite = overwrite
        request.fast_save = bool(fast_save)

        future = self.client.call_async(request)
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self)
            executor.shutdown()
        if not future.done():
            return False, "保存地图超时。", ""

        result = future.result()
        if result is None:
            return False, "保存地图失败，服务没有返回结果。", ""
        return bool(result.success), str(result.message), str(result.manifest_path)


class LoadMapClient(Node):
    def __init__(self) -> None:
        super().__init__(f"map_save_gui_load_client_{time.time_ns()}")
        self.client = self.create_client(
            LoadNavigationMapPackage, "/map_package_manager/load_package"
        )

    def load_package(self, package_path: str, timeout_sec: float | None = None):
        if timeout_sec is None:
            timeout_sec = load_navigation_map_client_spin_timeout_sec()
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, "读取服务 /map_package_manager/load_package 不可用。"

        request = LoadNavigationMapPackage.Request()
        request.package_path = package_path

        future = self.client.call_async(request)
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self)
            executor.shutdown()
        if not future.done():
            return False, "读取地图超时。"

        result = future.result()
        if result is None:
            return False, "读取地图失败，服务没有返回结果。"
        return bool(result.success), str(result.message)


class SavePackageThread(QThread):
    """在 QThread 内跑 ROS 保存，避免 threading.Thread + pyqtSignal 跨线程导致界面不回调。"""

    # 勿命名为 finished：会与 QThread 自带的 finished() 冲突，导致完成信号异常、界面一直 busy。
    save_done = pyqtSignal(bool, str, str)

    def __init__(
        self, package_path: str, overwrite: bool, fast_save: bool, parent: QWidget | None = None
    ) -> None:
        super().__init__(parent)
        self._package_path = package_path
        self._overwrite = overwrite
        self._fast_save = fast_save
        self._last_success = False
        self._last_message = ""
        self._last_manifest = ""

    def run(self) -> None:
        time.sleep(0.35)
        ok, message, manifest_path = False, "", ""
        node: SaveMapClient | None = None
        try:
            node = SaveMapClient()
            ok, message, manifest_path = node.save_package(
                self._package_path, self._overwrite, fast_save=self._fast_save
            )
        except Exception as exc:
            ok, message, manifest_path = False, f"保存过程异常：{exc}", ""
        finally:
            if node is not None:
                node.destroy_node()
        self._last_success = ok
        self._last_message = message
        self._last_manifest = manifest_path
        self.save_done.emit(ok, message, manifest_path)


class LoadPackageThread(QThread):
    load_done = pyqtSignal(bool, str)

    def __init__(self, package_path: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._package_path = package_path
        self._last_success = False
        self._last_message = ""

    def run(self) -> None:
        ok, message = False, ""
        node: LoadMapClient | None = None
        try:
            node = LoadMapClient()
            ok, message = node.load_package(self._package_path)
        except Exception as exc:
            ok, message = False, f"加载过程异常：{exc}"
        finally:
            if node is not None:
                node.destroy_node()
        self._last_success = ok
        self._last_message = message
        self.load_done.emit(ok, message)


class MapSaveWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._io_thread: QThread | None = None
        self._default_root = Path("/home/allen/maps")
        self._busy_elapsed = 0
        self._busy_op = "save"
        self._busy_path = ""
        self._busy_timer = QTimer(self)
        self._busy_timer.timeout.connect(self._tick_busy_status)
        self._io_finish_handled = False
        self._finish_poll_timer = QTimer(self)
        self._finish_poll_timer.setInterval(100)
        self._finish_poll_timer.timeout.connect(self._poll_io_thread_finish)
        self._init_ui()

    def _init_ui(self) -> None:
        self.setWindowTitle("地图保存")
        self.setMinimumWidth(560)

        root = QVBoxLayout()
        root.setSpacing(14)

        path_label = QLabel("根目录")
        path_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        path_row = QHBoxLayout()
        self.path_edit = QLineEdit()
        self.path_edit.setText(str(self._default_root))
        self.path_edit.setPlaceholderText("请选择地图根目录")
        browse_btn = QPushButton("选择文件夹")
        browse_btn.clicked.connect(self._choose_directory)
        path_row.addWidget(self.path_edit, 1)
        path_row.addWidget(browse_btn)

        name_label = QLabel("地图名")
        name_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("请输入地图名，例如 lv2_map_001")

        self.overwrite_checkbox = QCheckBox("允许覆盖已存在目录")
        self.overwrite_checkbox.setChecked(True)

        self.fast_save_checkbox = QCheckBox("快速保存（不重算禁行/可通行等，明显更快）")
        self.fast_save_checkbox.setChecked(True)
        self.fast_save_checkbox.setToolTip(
            "开启时仅同步 OctoMap 与当前话题上的图层并写盘，不在规划节点内完整重建派生图层。\n"
            "若刚改过楼梯/可行域等规划参数，需要与规划器完全一致时再保存，请取消勾选。"
        )

        tip = QLabel(
            "说明：本窗口将当前 ROS 中的 OctoMap 与图层写入地图包（调用 map_package_manager）。\n"
            "若在「地图查看」里做了栅格编辑，可先在该窗口自动保存或稍等片刻，再于此处点击「保存地图」。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color: palette(mid);")

        self.status_label = QLabel("等待操作。")
        self.status_label.setWordWrap(True)

        self.save_button = QPushButton("保存地图")
        self.save_button.clicked.connect(self._start_save)
        self.load_button = QPushButton("加载地图")
        self.load_button.clicked.connect(self._start_load)

        root.addWidget(path_label)
        root.addLayout(path_row)
        root.addWidget(name_label)
        root.addWidget(self.name_edit)
        root.addWidget(self.overwrite_checkbox)
        root.addWidget(self.fast_save_checkbox)
        root.addWidget(tip)
        root.addWidget(self.status_label)
        button_row = QHBoxLayout()
        button_row.addWidget(self.save_button)
        button_row.addWidget(self.load_button)
        root.addLayout(button_row)
        self.setLayout(root)

    def _choose_directory(self) -> None:
        initial_dir = self.path_edit.text().strip() or str(Path.home())
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图保存目录",
            initial_dir,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected:
            self.path_edit.setText(selected)

    def _set_busy(self, busy: bool) -> None:
        self.path_edit.setEnabled(not busy)
        self.name_edit.setEnabled(not busy)
        self.overwrite_checkbox.setEnabled(not busy)
        self.fast_save_checkbox.setEnabled(not busy)
        self.save_button.setEnabled(not busy)
        self.load_button.setEnabled(not busy)

    def _tick_busy_status(self) -> None:
        if self._io_thread is None or not self._io_thread.isRunning():
            self._busy_timer.stop()
            return
        self._busy_elapsed += 1
        if self._busy_op == "save":
            self.status_label.setText(
                f"正在保存地图到 {self._busy_path} ，已等待 {self._busy_elapsed} 秒…"
            )
        else:
            self.status_label.setText(
                f"正在加载地图 {self._busy_path} ，已等待 {self._busy_elapsed} 秒…"
            )

    def _poll_io_thread_finish(self) -> None:
        """若 save_done/load_done 未投递到主线程，则在线程结束后用缓存结果补一次完成逻辑。"""
        if self._io_finish_handled:
            self._finish_poll_timer.stop()
            return
        t = self._io_thread
        if t is None or t.isRunning():
            return
        self._finish_poll_timer.stop()
        if isinstance(t, SavePackageThread):
            self._on_save_finished(t._last_success, t._last_message, t._last_manifest)
        elif isinstance(t, LoadPackageThread):
            self._on_load_finished(t._last_success, t._last_message)

    def _build_package_path(self) -> Path | None:
        root_dir = self.path_edit.text().strip()
        map_name = self.name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "地图目录", "请先选择地图根目录。")
            return None
        if not map_name:
            QMessageBox.warning(self, "地图目录", "请输入地图名。")
            return None
        return Path(root_dir).expanduser() / map_name

    def _start_save(self) -> None:
        package_path = self._build_package_path()
        if package_path is None:
            return
        if self._io_thread is not None and self._io_thread.isRunning():
            QMessageBox.warning(self, "保存地图", "正在保存或加载，请完成后重试。")
            return

        overwrite = self.overwrite_checkbox.isChecked() or not package_path.exists()

        self._set_busy(True)
        self.status_label.setText(f"正在保存地图到 {package_path} ，请稍候。")
        self._busy_op = "save"
        self._busy_path = str(package_path)
        self._busy_elapsed = 0
        self._busy_timer.start(1000)

        self._io_finish_handled = False
        self._finish_poll_timer.start()
        self._io_thread = SavePackageThread(
            str(package_path), overwrite, self.fast_save_checkbox.isChecked(), self
        )
        self._io_thread.save_done.connect(self._on_save_finished, Qt.QueuedConnection)
        self._io_thread.start()

    def _start_load(self) -> None:
        if self._io_thread is not None and self._io_thread.isRunning():
            QMessageBox.warning(self, "加载地图", "正在保存或加载，请完成后重试。")
            return
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图目录",
            self.path_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if not selected:
            return
        selected_path = Path(selected)
        self.path_edit.setText(str(selected_path.parent))
        self.name_edit.setText(selected_path.name)

        self._set_busy(True)
        self.status_label.setText(f"正在加载地图 {selected_path} ，请稍候。")
        self._busy_op = "load"
        self._busy_path = str(selected_path)
        self._busy_elapsed = 0
        self._busy_timer.start(1000)

        self._io_finish_handled = False
        self._finish_poll_timer.start()
        self._io_thread = LoadPackageThread(str(selected_path), self)
        self._io_thread.load_done.connect(self._on_load_finished, Qt.QueuedConnection)
        self._io_thread.start()

    def _on_save_finished(self, success: bool, message: str, manifest_path: str) -> None:
        if self._io_finish_handled:
            return
        self._io_finish_handled = True
        self._finish_poll_timer.stop()
        self._busy_timer.stop()
        self._set_busy(False)
        self.status_label.setText(message)
        if success:
            body = "地图保存成功。"
            if manifest_path:
                body += f"\n\n清单路径：\n{manifest_path}"
            QMessageBox.information(self, "保存地图", body)
        else:
            QMessageBox.critical(self, "保存地图", message)

    def _on_load_finished(self, success: bool, message: str) -> None:
        if self._io_finish_handled:
            return
        self._io_finish_handled = True
        self._finish_poll_timer.stop()
        self._busy_timer.stop()
        self._set_busy(False)
        self.status_label.setText(message)
        if success:
            QMessageBox.information(self, "加载地图", "地图加载成功。")
        else:
            QMessageBox.critical(self, "加载地图", message)


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MapSaveWindow()
    window.show()
    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)
