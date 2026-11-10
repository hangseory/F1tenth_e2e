#!/usr/bin/env python3
"""panel_client.py — 원격 제어판 클라이언트 (노트북에서 돈다. 젯슨에서도 된다).

설계: docs/원격_제어판_설계.md

★ 이 파일에는 기능 지식이 하나도 없다.
  버튼 이름·명령·검증·경고 문구는 전부 서버에 있다. 여기가 아는 것은 네 가지뿐:
    위젯 어휘 / 메시지 형식 / 키 전송 규칙 / 하트비트
  그래서 젯슨에 모드를 추가해도 이 파일은 안 고친다. 고쳐야 하는 경우는 위젯
  어휘나 메시지 형식이 바뀔 때뿐이고, 그때는 서버의 PROTOCOL 을 올린다.

★ ROS 가 필요 없다. SSH 터널로만 통신한다 — ROS_DOMAIN_ID·subnet·멀티캐스트가
  관여하지 않는다. (2026-11-08~09 에 그 설정들 때문에 RViz 가 하루 종일 안 떴다)

쓰는 법
    노트북:  ssh -N -L 8765:127.0.0.1:8765 jetson@<IP> &
             ./panel.sh
"""
import json
import sys

from PyQt5 import QtCore, QtNetwork, QtWidgets

PROTOCOL = 3
HOST, PORT = '127.0.0.1', 8765
BEAT_MS = 100            # 10Hz. 서버 데드맨이 0.3s 를 본다


def nofocus(w):
    """스페이스바가 버튼을 누르지 않게 한다 — 키는 서버로 보내야 한다."""
    w.setFocusPolicy(QtCore.Qt.NoFocus)
    return w


class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('제어판 — 연결 중…')
        self.resize(560, 720)
        self.ids = {}                 # id -> widget
        self.tabs = None
        self.tab_of = {}              # 로그 탭 이름 -> QPlainTextEdit
        self.root = QtWidgets.QVBoxLayout(self)
        self.placeholder = QtWidgets.QLabel('서버에 연결 중…\n\nssh -N -L 8765:127.0.0.1:8765 jetson@<IP>')
        self.placeholder.setAlignment(QtCore.Qt.AlignCenter)
        self.root.addWidget(self.placeholder)

        self.buf = b''
        self.sock = QtNetwork.QTcpSocket(self)
        self.sock.readyRead.connect(self._read)
        self.sock.connected.connect(self._connected)
        self.sock.disconnected.connect(self._dropped)
        self.sock.errorOccurred.connect(lambda *_: self._dropped())

        self.beat = QtCore.QTimer(self)
        self.beat.timeout.connect(lambda: self.send({'op': 'heartbeat'}))
        self.retry = QtCore.QTimer(self)
        self.retry.timeout.connect(self._connect)
        self._connect()

    # ── 연결 ──────────────────────────────────────────────────────────────
    def _connect(self):
        if self.sock.state() == QtNetwork.QAbstractSocket.UnconnectedState:
            self.sock.connectToHost(HOST, PORT)

    def _connected(self):
        self.retry.stop()
        self.beat.start(BEAT_MS)
        self.setWindowTitle('제어판')

    def _dropped(self):
        self.beat.stop()
        self.setWindowTitle('제어판 — 연결 끊김, 재접속 중…')
        # ★ 창을 지우지 않는다. 마지막 화면을 그대로 두고 제목으로만 알린다.
        self.retry.start(1000)

    def send(self, msg):
        if self.sock.state() == QtNetwork.QAbstractSocket.ConnectedState:
            self.sock.write((json.dumps(msg, ensure_ascii=False) + '\n').encode())

    def _read(self):
        self.buf += bytes(self.sock.readAll())
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            if not line.strip():
                continue
            try:
                self.on_msg(json.loads(line.decode()))
            except Exception as exc:
                print('메시지 처리 실패:', exc, file=sys.stderr)

    # ── 메시지 ────────────────────────────────────────────────────────────
    def on_msg(self, m):
        op = m.get('op')
        if op == 'hello':
            if m.get('protocol') != PROTOCOL:
                self._fatal(f"제어판 업데이트 필요\n서버 v{m.get('protocol')} / 클라이언트 v{PROTOCOL}")
            self.setWindowTitle(m.get('title', '제어판'))
        elif op == 'layout':
            self.build(m['widgets'])
        elif op == 'update':
            w = self.ids.get(m['id'])
            if isinstance(w, QtWidgets.QLabel):
                w.setText(m.get('text', ''))
            elif isinstance(w, QtWidgets.QPushButton):
                w.setText(m.get('text', w.text()))
            if m['id'] == 'status' and self.statusbar is not None:
                self.statusbar.setText(m.get('text', ''))
        elif op == 'log':
            self.log(m.get('tab', 'log'), m.get('line', ''))
        elif op == 'alert':
            QtWidgets.QMessageBox.warning(self, '알림', m.get('text', ''))

    def _fatal(self, text):
        self._clear()
        lb = QtWidgets.QLabel(text)
        lb.setAlignment(QtCore.Qt.AlignCenter)
        self.root.addWidget(lb)
        self.sock.abort()
        self.retry.stop()

    # ── 레이아웃 그리기 (위젯 어휘) ────────────────────────────────────────
    def _clear(self):
        while self.root.count():
            it = self.root.takeAt(0)
            if it.widget():
                it.widget().deleteLater()
        self.ids.clear()
        self.tab_of.clear()
        self.tabs = None
        self.statusbar = None

    def build(self, widgets):
        self._clear()
        for w in widgets:
            self.root.addWidget(self._make(w))

    def _make(self, spec):
        t = spec.get('t')
        if t == 'group':
            box = QtWidgets.QGroupBox(spec.get('text', ''))
            v = QtWidgets.QVBoxLayout(box)
            for c in spec.get('children', []):
                v.addWidget(self._make(c))
            return box
        if t == 'row':
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            for c in spec.get('children', []):
                h.addWidget(self._make(c))
            return w
        if t == 'label':
            lb = QtWidgets.QLabel(spec.get('text', ''))
            lb.setWordWrap(True)
            self.ids[spec['id']] = lb
            return lb
        if t == 'button':
            b = nofocus(QtWidgets.QPushButton(spec.get('text', '')))
            if spec.get('color'):
                b.setStyleSheet(f"background:{spec['color']}; color:white; font-weight:bold;")
            b.clicked.connect(lambda _=False, i=spec['id']: self.send(
                {'op': 'action', 'id': i, 'value': None}))
            self.ids[spec['id']] = b
            return b
        if t == 'radio':
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            grp = QtWidgets.QButtonGroup(w)
            for key, label, tip in spec.get('options', []):
                r = nofocus(QtWidgets.QRadioButton(label))
                r.setToolTip(tip or '')
                r.setChecked(key == spec.get('value'))
                r.toggled.connect(lambda on, k=key, i=spec['id']: on and self.send(
                    {'op': 'action', 'id': i, 'value': k}))
                grp.addButton(r)
                h.addWidget(r)
            h.addStretch(1)
            self.ids[spec['id']] = w
            return w
        if t == 'lineedit':
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            if spec.get('label'):
                lb = QtWidgets.QLabel(spec['label'])
                lb.setMinimumWidth(70)
                h.addWidget(lb)
            le = QtWidgets.QLineEdit(spec.get('value', ''))
            le.setPlaceholderText(spec.get('placeholder', ''))
            # 타이핑 한 글자마다 보내지 않는다 — 서버가 매번 검증하면 시끄럽다.
            # 포커스를 잃거나 Enter 를 칠 때만 보낸다.
            le.editingFinished.connect(
                lambda i=spec['id'], e=le: self.send(
                    {'op': 'action', 'id': i, 'value': e.text()}))
            h.addWidget(le, 1)
            self.ids[spec['id']] = le
            return w
        if t == 'combo':
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            if spec.get('label'):
                lb = QtWidgets.QLabel(spec['label'])
                lb.setMinimumWidth(80)
                h.addWidget(lb)
            cb = nofocus(QtWidgets.QComboBox())
            cb.addItems([str(o) for o in spec.get('options', [])])
            v = spec.get('value')
            if v is not None:
                i = cb.findText(str(v))
                # 현재 값이 목록에 없으면 맨 앞에 넣어 보여준다 — 조용히 다른
                # 값으로 바뀐 것처럼 보이면 안 된다 (설정이 깨진 상태의 신호다)
                if i < 0:
                    cb.insertItem(0, str(v))
                    i = 0
                cb.setCurrentIndex(i)
            cb.currentTextChanged.connect(
                lambda txt, i=spec['id']: self.send({'op': 'action', 'id': i, 'value': txt}))
            h.addWidget(cb, 1)
            self.ids[spec['id']] = cb
            return w
        if t == 'spin':
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            if spec.get('label'):
                lb = QtWidgets.QLabel(spec['label'])
                lb.setMinimumWidth(140)
                h.addWidget(lb)
            sp = nofocus(QtWidgets.QDoubleSpinBox())
            sp.setRange(float(spec.get('min', 0.0)), float(spec.get('max', 10.0)))
            sp.setSingleStep(float(spec.get('step', 0.1)))
            sp.setDecimals(2)
            sp.setSuffix(spec.get('suffix', ''))
            try:
                sp.setValue(float(spec.get('value', 0)))
            except (TypeError, ValueError):
                pass
            # editingFinished 를 쓴다 — valueChanged 면 스핀 한 칸마다 tune.sh 가
            # 돌아 느리고, 중간값이 파일에 써진다
            sp.editingFinished.connect(
                lambda i=spec['id'], s_=sp: self.send(
                    {'op': 'action', 'id': i, 'value': round(s_.value(), 3)}))
            h.addWidget(sp, 1)
            self.ids[spec['id']] = sp
            return w
        if t == 'logtabs':
            self.tabs = QtWidgets.QTabWidget()
            self.ids[spec['id']] = self.tabs
            return self.tabs
        if t == 'statusline':
            self.statusbar = QtWidgets.QLabel('')
            self.statusbar.setWordWrap(True)
            self.statusbar.setStyleSheet('color:#555;')
            self.ids[spec['id']] = self.statusbar
            return self.statusbar
        return QtWidgets.QLabel(f'(모르는 위젯: {t})')

    def log(self, tab, line):
        if self.tabs is None:
            return
        te = self.tab_of.get(tab)
        if te is None:
            te = QtWidgets.QPlainTextEdit(readOnly=True)
            te.setMaximumBlockCount(2000)
            te.setStyleSheet('font-family: monospace; font-size: 11px;')
            self.tab_of[tab] = te
            self.tabs.addTab(te, tab)
        te.appendPlainText(line)

    # ── 키 ────────────────────────────────────────────────────────────────
    def keyPressEvent(self, e):
        if not e.isAutoRepeat():
            self.send({'op': 'key', 'key': e.text() or str(e.key()), 'down': True})

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.send({'op': 'key', 'key': e.text() or str(e.key()), 'down': False})


def main():
    app = QtWidgets.QApplication(sys.argv)
    p = Panel()
    p.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
