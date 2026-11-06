#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 펌웨어 업로더 — 라즈베리파이에서 esp32_controller.ino를 컴파일해 ESP32에 올린다.

아두이노 IDE 없이, ESP32를 라즈베리파이 USB에 꽂은 채로 실행하면 된다:

    python3 upload_firmware.py

이 파일 하나로 단독 실행된다. 라즈베리파이 OS에 기본으로 있는 python3(표준 라이브러리)만
쓰고, 그 밖에 필요한 것(arduino-cli, ESP32 코어, 라이브러리)은 스크립트가 직접 내려받는다.
robot_dog 폴더의 코드나 pyserial도 필요 없다. 인터넷은 최초 1회 설치 때만 필요하다.

하는 일 (필요한 것만 자동으로 수행):
  1. ESP32 포트 찾기        USB-시리얼 포트 자동 탐색 (없으면 GPIO UART /dev/ttyAMA2)
  2. arduino-cli 준비       없으면 ~/.local/bin 에 내려받는다
  3. ESP32 코어/라이브러리  없으면 설치한다 (최초 1회, 디스크 약 6GB · 수십 분)
  4. 컴파일                 결과는 ~/.cache/robodog_esp32_build 에 캐시 (2회차부터 빠름)
  5. 업로드 + 부팅 확인     포트를 쓰는 프로그램(flask_web.py 등)을 종료하고 업로드,
                            전송 오류 시 속도를 낮춰 재시도, 끝나면 FIRMWARE_VER 확인

자주 쓰는 옵션:
    --port /dev/ttyUSB0     포트 직접 지정 (기본: 자동 탐색)
    --baud 115200           업로드 속도 고정 (기본: 460800 -> 실패 시 115200)
    --compile-only          컴파일만 (보드 없이 코드 오류 확인)
    --list-ports            후보 포트만 출력
    -y                      질문 없이 진행 (포트를 쓰는 프로그램 자동 종료)
    -v                      컴파일/업로드 상세 로그

업로드가 "Failed to connect"로 실패하면 보드를 수동으로 다운로드 모드에 넣어야 한다:
BOOT 버튼을 누른 채 EN(RST)을 한 번 눌렀다 뗀다 (스크립트가 그때 안내한다).
"""
import argparse
import glob
import os
import platform
import queue
import re
import select
import shutil
import signal
import stat
import struct
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import urllib.request

# ==================== 설정 ====================
SKETCH_DIR = os.path.dirname(os.path.abspath(__file__))
SKETCH_FILE = os.path.join(SKETCH_DIR, os.path.basename(SKETCH_DIR) + ".ino")

FQBN = "esp32:esp32:esp32"       # ESP32 Dev Module (스케치 머리말의 보드 설정과 동일)
CORE = "esp32:esp32"
CORE_VERSION = "3.3.10"          # 새로 설치할 때의 버전 (PC 아두이노 IDE에 설치된 것과 같은 버전)
CORE_DISK_GB = 8                 # 코어 설치에 필요한 여유 공간 (설치 약 6GB + 내려받기 임시 파일)
LIBRARIES = ["Adafruit SSD1306", "Adafruit GFX Library"]  # 의존 라이브러리(BusIO)는 자동 설치됨

CLI_VERSION = "1.5.1"            # 자동 설치할 arduino-cli 버전
CLI_MIN_VERSION = (1, 0, 0)      # --upload-property 를 쓰므로 1.0 이상 필요
LOCAL_CLI = os.path.join(os.path.expanduser("~"), ".local", "bin", "arduino-cli")
BUILD_DIR = os.path.join(os.path.expanduser("~"), ".cache", "robodog_esp32_build")

UPLOAD_SPEEDS = [460800, 115200]  # 앞에서부터 시도, 전송 오류 시 다음 속도로 재시도
SERIAL_BAUD = 115200              # 펌웨어 USB 시리얼 속도 (부팅 확인용)
GPIO_UART = "/dev/ttyAMA2"        # USB가 아닌 GPIO UART 연결 (robot_dog/util.py와 같은 후보)
# USB-시리얼 칩 제조사 ID: CP210x, CH340/CH9102, FTDI, Espressif 내장 USB
ESP_USB_VIDS = {0x10C4, 0x1A86, 0x0403, 0x303A}

# arduino-cli 실행 환경. 기본 네트워크 제한시간(1분)으로는 ESP32 코어(수백 MB)를
# 내려받다가 "context deadline exceeded"로 끊기므로 넉넉히 늘린다.
CLI_ENV = dict(os.environ)
CLI_ENV.setdefault("ARDUINO_NETWORK_CONNECTION_TIMEOUT", "3600s")

MANUAL_BOOT_HELP = (
    "ESP32를 수동으로 다운로드 모드에 넣어 주세요:\n"
    "      1) BOOT 버튼을 누른 채로\n"
    "      2) EN(RST) 버튼을 한 번 눌렀다 떼고\n"
    "      3) BOOT 버튼에서 손을 뗍니다"
)


class Fail(Exception):
    """사용자에게 메시지를 보여 주고 중단해야 하는 오류"""


# ==================== 출력 / 실행 도우미 ====================

_step = {"n": 0, "total": 5}


def step(title):
    _step["n"] += 1
    print(f"\n== [{_step['n']}/{_step['total']}] {title} ==", flush=True)


def info(msg):
    print("  " + msg, flush=True)


def warn(msg):
    print("  [주의] " + msg, flush=True)


def fmt_elapsed(sec):
    sec = int(sec)
    return f"{sec // 60}분 {sec % 60}초" if sec >= 60 else f"{sec}초"


def ask(question, assume_yes):
    """예/아니오 질문. -y면 항상 예, 입력을 받을 수 없는 환경이면 아니오."""
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        return False
    try:
        ans = input(f"  {question} [Y/n] ").strip().lower()
    except EOFError:
        return False
    return ans in ("", "y", "yes", "ㅛ", "예", "네")  # ㅛ = 한글 입력 상태의 y


def run(cmd, show=True, heartbeat=0):
    """
    명령을 실행하고 (종료 코드, 전체 출력)을 반환한다.
    show=True면 출력을 실시간으로 보여 준다. heartbeat(초) 동안 출력이 없으면
    경과 시간을 찍어, 오래 걸리는 설치/컴파일이 멈춘 것처럼 보이지 않게 한다.
    """
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=CLI_ENV,
                                encoding="utf-8", errors="replace")
    except OSError as e:
        raise Fail(f"실행 실패: {cmd[0]} ({e})")

    lines = queue.Queue()

    def pump():
        for line in proc.stdout:  # \r 로 끝나는 진행률 갱신도 한 줄로 들어온다
            lines.put(line)
        lines.put(None)

    threading.Thread(target=pump, daemon=True).start()

    output = []
    start = last = time.time()
    tty = sys.stdout.isatty()
    inplace = False  # 직전 줄을 제자리(\r) 진행률로 출력했는지
    while True:
        try:
            line = lines.get(timeout=1.0)
        except queue.Empty:
            if show and heartbeat and time.time() - last >= heartbeat:
                last = time.time()
                if inplace:
                    print()
                    inplace = False
                print(f"    ... 진행 중 ({fmt_elapsed(last - start)} 경과)", flush=True)
            continue
        if line is None:
            break
        last = time.time()
        output.append(line)
        text = line.rstrip()
        if not show or not text:
            continue
        # esptool 진행률("Writing at 0x... (42 %)")은 터미널이면 한 줄에서 갱신한다
        if tty and "%" in text and text.lstrip().startswith("Writing at"):
            sys.stdout.write("\r    " + text[:100].ljust(100))
            sys.stdout.flush()
            inplace = True
            continue
        if inplace:
            print()
            inplace = False
        print("    " + text, flush=True)
    if inplace:
        print()
    proc.wait()
    return proc.returncode, "".join(output)


# ==================== arduino-cli 준비 ====================

def cli_version(path):
    """arduino-cli 버전 튜플. 실행할 수 없으면 None."""
    try:
        out = subprocess.run([path, "version"], capture_output=True, timeout=30,
                             encoding="utf-8", errors="replace").stdout or ""
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"Version:\s*(\d+)\.(\d+)\.(\d+)", out)
    if m:
        return tuple(int(x) for x in m.groups())
    # nightly 등 숫자 버전이 아닌 빌드는 최신으로 본다
    return (99, 0, 0) if "arduino-cli" in out.lower() else None


def cli_asset():
    """이 시스템에 맞는 arduino-cli 배포 파일 이름 (자동 설치는 리눅스만 지원)"""
    if not sys.platform.startswith("linux"):
        return None
    machine = platform.machine().lower()
    # 64비트 커널 + 32비트 OS(userland)면 툴체인도 32비트여야 실행된다
    is_32bit_os = struct.calcsize("P") == 4
    if machine in ("aarch64", "arm64"):
        return "Linux_ARMv7" if is_32bit_os else "Linux_ARM64"
    if machine.startswith("armv7") or machine.startswith("armv8"):
        return "Linux_ARMv7"
    if machine.startswith("armv6"):
        return "Linux_ARMv6"
    if machine in ("x86_64", "amd64"):
        return "Linux_64bit"
    if machine in ("i386", "i686"):
        return "Linux_32bit"
    return None


def install_cli():
    """arduino-cli를 ~/.local/bin 에 내려받아 설치하고 경로를 반환"""
    asset = cli_asset()
    if asset is None:
        raise Fail(
            f"이 시스템({platform.system()} {platform.machine()})은 arduino-cli 자동 설치를 "
            "지원하지 않습니다.\n  직접 설치한 뒤 --arduino-cli <경로> 로 지정하세요: "
            "https://arduino.github.io/arduino-cli/latest/installation/")
    url = f"https://downloads.arduino.cc/arduino-cli/arduino-cli_{CLI_VERSION}_{asset}.tar.gz"
    info(f"arduino-cli {CLI_VERSION} 내려받는 중... ({asset})")
    os.makedirs(os.path.dirname(LOCAL_CLI), exist_ok=True)
    tmp_bin = LOCAL_CLI + ".download"
    try:
        with tempfile.TemporaryDirectory() as tmp:
            archive = os.path.join(tmp, "arduino-cli.tar.gz")
            req = urllib.request.Request(url, headers={"User-Agent": "robodog-upload-firmware"})
            with urllib.request.urlopen(req, timeout=60) as resp, open(archive, "wb") as f:
                shutil.copyfileobj(resp, f)
            with tarfile.open(archive) as tar:
                member = tar.extractfile("arduino-cli")  # 실행 파일 하나만 꺼낸다
                with open(tmp_bin, "wb") as f:
                    shutil.copyfileobj(member, f)
        os.chmod(tmp_bin, 0o755)
        os.replace(tmp_bin, LOCAL_CLI)
    except (OSError, tarfile.TarError, KeyError) as e:  # URLError는 OSError의 하위 클래스
        if os.path.exists(tmp_bin):
            os.remove(tmp_bin)
        raise Fail(f"arduino-cli 내려받기 실패: {e}\n  인터넷 연결을 확인하세요. ({url})")
    if cli_version(LOCAL_CLI) is None:
        raise Fail(f"설치한 arduino-cli가 실행되지 않습니다: {LOCAL_CLI}")
    return LOCAL_CLI


def ensure_cli(explicit):
    """사용할 arduino-cli 경로를 반환 (없거나 너무 오래됐으면 설치)"""
    if explicit:
        if cli_version(explicit) is None:
            raise Fail(f"--arduino-cli 로 지정한 파일을 실행할 수 없습니다: {explicit}")
        cli = explicit
    else:
        cli = None
        for cand in (shutil.which("arduino-cli"), LOCAL_CLI):
            if cand and os.path.isfile(cand):
                ver = cli_version(cand)
                if ver and ver >= CLI_MIN_VERSION:
                    cli = cand
                    break
        if cli is None:
            cli = install_cli()
    ver = cli_version(cli)
    ver_text = ".".join(map(str, ver)) if ver and ver[0] != 99 else "(개발 빌드)"
    info(f"arduino-cli {ver_text}: {cli}")
    return cli


# ==================== ESP32 코어 / 라이브러리 ====================

def installed_core_version(cli):
    rc, out = run([cli, "core", "list"], show=False)
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == CORE:
            return parts[1]
    return None


def ensure_core(cli, wanted, assume_yes):
    """ESP32 코어 확인. 이미 설치돼 있으면 그대로 쓰고(버전을 지정한 경우만 교체), 없으면 설치."""
    have = installed_core_version(cli)
    if have and wanted in (None, have):
        info(f"ESP32 코어 {have}: 설치됨")
        return
    target = wanted or CORE_VERSION

    free_gb = shutil.disk_usage(os.path.expanduser("~")).free / 1e9
    if free_gb < CORE_DISK_GB:
        warn(f"디스크 여유 공간이 {free_gb:.1f}GB입니다. ESP32 코어 설치에는 약 {CORE_DISK_GB}GB가 필요합니다.")
        if not ask("그래도 설치를 계속할까요?", assume_yes):
            raise Fail("디스크 공간을 확보한 뒤 다시 실행하세요.")

    info(f"ESP32 코어 {target} 설치 중... (최초 1회. 내려받는 양이 많아 수십 분 걸릴 수 있습니다)")
    rc, _ = run([cli, "core", "update-index"], heartbeat=30)
    if rc != 0:
        raise Fail("보드 목록을 내려받지 못했습니다. 인터넷 연결을 확인하세요.")
    rc, _ = run([cli, "core", "install", f"{CORE}@{target}"], heartbeat=30)
    if rc != 0:
        raise Fail("ESP32 코어 설치 실패. 위 메시지를 확인한 뒤 다시 실행하면 이어서 설치합니다.")
    run([cli, "cache", "clean"], show=False)  # 내려받은 압축 파일 삭제 (디스크 1GB 이상 확보)
    info(f"ESP32 코어 {target}: 설치 완료")


def ensure_libs(cli):
    rc, out = run([cli, "lib", "list"], show=False)
    # 출력 형식: "<이름> <버전> ..." (이름에 공백이 있어 줄 시작 + 버전 숫자로 판별)
    missing = [lib for lib in LIBRARIES
               if not re.search(r"^%s\s+\d" % re.escape(lib), out, re.M)]
    if not missing:
        info("라이브러리: 설치됨 (" + ", ".join(LIBRARIES) + ")")
        return
    info("라이브러리 설치 중: " + ", ".join(missing))
    run([cli, "lib", "update-index"], show=False)
    rc, _ = run([cli, "lib", "install"] + missing, heartbeat=30)
    if rc != 0:
        raise Fail("라이브러리 설치 실패. 인터넷 연결을 확인하세요.")


# ==================== 컴파일 ====================

def compile_sketch(cli, fqbn, build_dir, verbose):
    os.makedirs(build_dir, exist_ok=True)
    cmd = [cli, "compile", "--fqbn", fqbn, "--build-path", build_dir]
    if verbose:
        cmd.append("-v")
    cmd.append(SKETCH_DIR)
    info("컴파일 중... (첫 컴파일은 몇 분 걸립니다. 이후에는 캐시로 빨라집니다)")
    start = time.time()
    rc, _ = run(cmd, heartbeat=20)
    if rc != 0:
        raise Fail("컴파일 실패. 위 오류 메시지를 확인하세요.")
    info(f"컴파일 완료 ({fmt_elapsed(time.time() - start)})")


# ==================== 시리얼 포트 ====================

def usb_info(tty_name):
    """리눅스 sysfs에서 tty의 USB 정보 (vid, pid, 제품명)를 읽는다. 알 수 없으면 None."""
    path = os.path.realpath(f"/sys/class/tty/{tty_name}/device")
    for _ in range(6):  # 시리얼 포트 -> USB 인터페이스 -> USB 장치 순으로 올라가며 찾는다
        if os.path.isfile(os.path.join(path, "idVendor")):
            def read(name):
                try:
                    with open(os.path.join(path, name)) as f:
                        return f.read().strip()
                except OSError:
                    return ""
            try:
                return int(read("idVendor"), 16), int(read("idProduct"), 16), read("product")
            except ValueError:
                return None
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def list_ports():
    """
    ESP32 후보 포트 목록: USB-시리얼 포트 전부 + GPIO UART(/dev/ttyAMA2).
    robot_dog/util.py의 find_serial_candidates()와 같은 대상(USB 시리얼, 블루투스 제외)을
    찾되, 단독 실행을 위해 pyserial 없이 리눅스 장치 목록에서 직접 읽는다.
    각 항목: {"device", "desc", "usb"(자동 리셋 가능), "vid", "pid"}
    """
    ports = []
    if sys.platform.startswith("linux"):
        for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
            usb = usb_info(os.path.basename(dev))
            vid, pid, name = usb if usb else (None, None, "")
            ports.append({"device": dev, "desc": name or "USB serial", "usb": True,
                          "vid": vid, "pid": pid})
    else:
        # 리눅스가 아닌 PC에서 시험할 때만: pyserial이 있으면 그 목록을 쓴다 (없으면 --port 지정)
        try:
            from serial.tools import list_ports as lp
            for p in lp.comports():
                if "Bluetooth" in (p.description or "") or "BTHENUM" in (p.hwid or ""):
                    continue
                if p.vid is None:
                    continue
                ports.append({"device": p.device, "desc": p.description or "", "usb": True,
                              "vid": p.vid, "pid": p.pid})
        except ImportError:
            pass
    ports.sort(key=lambda p: (p["vid"] not in ESP_USB_VIDS, p["device"]))
    if os.path.exists(GPIO_UART):
        ports.append({"device": GPIO_UART, "desc": "GPIO UART (자동 리셋 없음: BOOT 버튼 필요)",
                      "usb": False, "vid": None, "pid": None})
    return ports


def describe_port(p):
    usb_id = f"  [USB {p['vid']:04X}:{p['pid']:04X}]" if p["vid"] is not None else ""
    return f"{p['device']}  {p['desc']}{usb_id}"


def find_port(explicit):
    """업로드할 포트 결정 -> (장치 경로, 자동 리셋 가능 여부)"""
    ports = list_ports()
    if explicit:
        if os.name != "nt" and not os.path.exists(explicit):
            raise Fail(f"{explicit} 포트가 없습니다. --list-ports 로 후보를 확인하세요.")
        real = explicit if os.name == "nt" else os.path.realpath(explicit)
        for p in ports:
            if p["device"] == real:
                return explicit, p["usb"]
        # 목록에 없는 포트: 내장 UART 이름이 아니면 USB(자동 리셋 가능)로 본다
        return explicit, not re.search(r"tty(AMA|S)\d|serial\d", real)

    usb = [p for p in ports if p["usb"]]
    known = [p for p in usb if p["vid"] in ESP_USB_VIDS]
    cands = known if len(known) == 1 else usb
    if len(cands) == 1:
        info("포트: " + describe_port(cands[0]))
        return cands[0]["device"], True
    if len(cands) > 1:
        info("ESP32 후보 포트가 여러 개입니다:")
        for i, p in enumerate(cands, 1):
            info(f"  {i}) {describe_port(p)}")
        if not sys.stdin.isatty():
            raise Fail("--port 로 사용할 포트를 지정하세요.")
        while True:
            ans = input(f"  번호 선택 [1-{len(cands)}]: ").strip()
            if ans.isdigit() and 1 <= int(ans) <= len(cands):
                return cands[int(ans) - 1]["device"], True
    gpio = [p for p in ports if not p["usb"]]
    if gpio:
        info("포트: " + describe_port(gpio[0]))
        return gpio[0]["device"], False
    raise Fail("ESP32 포트를 찾지 못했습니다.\n"
               "  - USB 케이블이 데이터 통신용인지(충전 전용 아님), 보드 전원이 켜졌는지 확인하세요.\n"
               "  - 포트를 알고 있다면 --port /dev/ttyUSB0 처럼 직접 지정할 수 있습니다.")


def port_holders(port):
    """이 포트를 열고 있는 다른 프로세스 [(pid, 명령줄)] (리눅스 전용, 그 외에는 빈 목록)"""
    if not sys.platform.startswith("linux"):
        return []
    try:
        target = os.stat(port).st_rdev
    except OSError:
        return []
    holders = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue  # 다른 사용자의 프로세스이거나 그새 종료됨
        for fd in fds:
            try:
                st = os.stat(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if stat.S_ISCHR(st.st_mode) and st.st_rdev == target:
                try:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                except OSError:
                    cmd = "?"
                holders.append((int(pid), cmd))
                break
    return holders


def free_port(port, assume_yes):
    """
    포트를 쓰는 프로그램(웹 서버 등)을 종료한다. 리눅스는 여러 프로그램이 같은 포트를
    동시에 열 수 있어서, 그대로 두면 업로드 데이터를 서로 가로채 업로드가 깨진다.
    종료한 프로세스 목록을 반환한다.
    """
    holders = port_holders(port)
    if not holders:
        return []
    info("이 포트를 사용 중인 프로그램이 있습니다 (업로드하려면 종료해야 합니다):")
    for pid, cmd in holders:
        info(f"  PID {pid}: {cmd}")
    if not ask("종료하고 계속할까요?", assume_yes):
        raise Fail("포트가 사용 중이라 업로드할 수 없습니다. 위 프로그램을 종료한 뒤 다시 실행하세요.")

    def send(sig):
        for pid, _ in port_holders(port):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                raise Fail(f"PID {pid}를 종료할 권한이 없습니다. 'sudo kill {pid}' 후 다시 실행하세요.")

    send(signal.SIGTERM)
    deadline = time.time() + 5
    while time.time() < deadline and port_holders(port):
        time.sleep(0.2)
    if port_holders(port):
        send(signal.SIGKILL)
        time.sleep(1.0)
    if port_holders(port):
        raise Fail("포트를 쓰는 프로그램이 종료되지 않습니다. 직접 종료한 뒤 다시 실행하세요.")
    info("종료 완료.")
    return holders


# ==================== 업로드 ====================

def classify_upload_error(out):
    """esptool 실패 원인 분류: permission / port / connect / transfer"""
    low = out.lower()
    if "permission denied" in low:
        return "permission"
    if "failed to connect" in low or "no serial data received" in low or "wrong boot mode" in low:
        return "connect"
    if "could not open" in low or "port is busy" in low or "no such file" in low:
        return "port"
    return "transfer"


def upload(cli, fqbn, port, build_dir, speeds, auto_reset, assume_yes, verbose):
    """업로드. 성공하면 True. 전송 오류는 다음(낮은) 속도로, 연결 실패는 수동 BOOT 안내 후 재시도."""
    interactive = sys.stdin.isatty() and not assume_yes
    manual_tried = False
    if not auto_reset:
        # GPIO UART에는 자동 리셋 신호선(DTR/RTS)이 없어 수동으로 다운로드 모드에 넣어야 한다
        info(MANUAL_BOOT_HELP)
        if interactive:
            input("  준비되면 Enter를 누르세요... ")
            manual_tried = True

    todo = list(speeds)
    while todo:
        speed = todo.pop(0)
        info(f"업로드 중... ({port}, {speed} baud)")
        cmd = [cli, "upload", "-p", port, "--fqbn", fqbn, "--input-dir", build_dir,
               "--upload-property", f"upload.speed={speed}"]
        if verbose:
            cmd.append("-v")
        cmd.append(SKETCH_DIR)
        rc, out = run(cmd)
        if rc == 0:
            return True

        kind = classify_upload_error(out)
        if kind == "permission":
            warn(f"{port} 접근 권한이 없습니다. 아래 명령 실행 후 다시 로그인(또는 재부팅)하세요:\n"
                 "      sudo usermod -aG dialout $USER")
            return False
        if kind == "port":
            warn(f"{port} 포트를 열 수 없습니다. 케이블 연결과 포트 이름을 확인하세요 (--list-ports).")
            return False
        if kind == "connect":
            if interactive and not manual_tried:
                manual_tried = True
                warn("ESP32가 다운로드 모드로 들어가지 않았습니다.")
                info(MANUAL_BOOT_HELP)
                input("  준비되면 Enter를 누르세요... ")
                todo.insert(0, speed)
                continue
            warn("ESP32 부트로더에 연결하지 못했습니다.")
            info(MANUAL_BOOT_HELP)
            info("그 상태에서 이 스크립트를 다시 실행하세요.")
            return False
        if todo:
            warn(f"전송 중 오류가 났습니다. 속도를 {todo[0]} baud로 낮춰 다시 시도합니다.")
    return False


class PosixSerial:
    """
    표준 라이브러리(termios)만으로 여는 시리얼 포트. pyserial 없이도 부팅 확인이 되게 한다.
    설정은 robot_dog/util.py의 connect_esp32()와 같다: 115200 8N1, 흐름 제어 없음.
    """

    def __init__(self, port, baud):
        import fcntl
        import termios
        self._fcntl = fcntl
        self._termios = termios
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0  # iflag: 입력 변환 없음 (받은 바이트 그대로)
            attrs[1] = 0  # oflag: 출력 변환 없음
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1, 수신 켬
            attrs[3] = 0  # lflag: 에코/줄 편집 없음
            attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            os.close(self.fd)
            raise

    def _set_line(self, bit, on):
        try:
            request = self._termios.TIOCMBIS if on else self._termios.TIOCMBIC
            self._fcntl.ioctl(self.fd, request, struct.pack("I", bit))
        except OSError:
            pass  # 제어선이 없는 포트 (GPIO UART 등)

    def set_dtr(self, on):
        self._set_line(self._termios.TIOCM_DTR, on)

    def set_rts(self, on):
        self._set_line(self._termios.TIOCM_RTS, on)

    def read(self, timeout):
        """timeout(초) 안에 들어온 데이터를 반환. 없으면 b''."""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return b""
        try:
            return os.read(self.fd, 256)
        except BlockingIOError:
            return b""

    def write(self, data):
        os.write(self.fd, data)

    def flush_input(self):
        self._termios.tcflush(self.fd, self._termios.TCIFLUSH)

    def close(self):
        os.close(self.fd)


def verify_boot(port, auto_reset):
    """
    업로드 후 새 펌웨어가 부팅됐는지 확인한다. 보드를 한 번 리셋해 부팅 메시지
    (FIRMWARE_VER ...)를 읽고, 놓쳤으면 설정을 건드리지 않는 V(목표각 읽기)로 응답을 본다.
    확인되면 True, 안 되면 False, 확인 자체를 못 하면 None.
    """
    try:
        ser = PosixSerial(port, SERIAL_BAUD)
    except ImportError:
        info("이 OS에서는 부팅 확인을 지원하지 않아 건너뜁니다.")
        return None
    except Exception as e:
        warn(f"부팅 확인용 포트 열기 실패: {e}")
        return None
    try:
        if auto_reset:
            # EN을 잠깐 내렸다 올린다 (DTR은 놓아 둬 IO0=HIGH -> 일반 부팅)
            ser.set_dtr(False)
            ser.set_rts(True)
            time.sleep(0.12)
            ser.set_rts(False)
            wait = 8
        else:
            info("보드의 EN(RST) 버튼을 눌러 재시작하세요. (최대 30초 기다립니다)")
            wait = 30

        text = ""
        deadline = time.time() + wait
        while time.time() < deadline:
            chunk = ser.read(0.2)
            if not chunk:
                continue
            text += chunk.decode("ascii", errors="ignore")
            m = re.search(r"FIRMWARE_VER:[^\r\n]*[\r\n]", text)
            if m:
                info("부팅 확인: " + m.group(0).strip())
                return True

        if "waiting for download" in text:
            warn("보드가 아직 다운로드 모드입니다. BOOT 버튼에서 손을 떼고 EN(RST)을 다시 누르세요.")
            return False

        # 부팅 메시지를 놓쳤을 수 있다 -> 목표각 읽기(V)로 펌웨어가 응답하는지 확인
        ser.flush_input()
        ser.write(b"V\n")
        reply = ""
        deadline = time.time() + 1.5
        while time.time() < deadline:
            reply += ser.read(0.2).decode("ascii", errors="ignore")
            if re.search(r"ANG[^\r\n]*[\r\n]", reply):
                info("펌웨어 응답 확인 (V 명령에 ANG 응답)")
                return True
        warn("업로드는 끝났지만 펌웨어 응답을 확인하지 못했습니다. EN(RST) 버튼을 눌러 재시작해 보세요.")
        return False
    finally:
        ser.close()


# ==================== 메인 ====================

def parse_args():
    ap = argparse.ArgumentParser(
        description="라즈베리파이에서 esp32_controller.ino를 컴파일해 ESP32에 업로드한다.")
    ap.add_argument("--port", help="시리얼 포트 (기본: 자동 탐색). 예: /dev/ttyUSB0")
    ap.add_argument("--baud", type=int,
                    help=f"업로드 속도 고정 (기본: {' -> '.join(map(str, UPLOAD_SPEEDS))} 순서로 시도)")
    ap.add_argument("--compile-only", action="store_true", help="컴파일만 하고 업로드하지 않는다")
    ap.add_argument("--list-ports", action="store_true", help="ESP32 후보 포트를 출력하고 끝낸다")
    ap.add_argument("-y", "--yes", action="store_true",
                    help="질문 없이 진행 (포트를 쓰는 프로그램 자동 종료)")
    ap.add_argument("-v", "--verbose", action="store_true", help="컴파일/업로드 상세 로그")
    ap.add_argument("--no-verify", action="store_true", help="업로드 후 부팅 확인을 건너뛴다")
    ap.add_argument("--fqbn", default=FQBN, help=f"보드 FQBN (기본: {FQBN})")
    ap.add_argument("--core-version",
                    help=f"ESP32 코어 버전 지정 (기본: 설치된 버전 사용, 없으면 {CORE_VERSION} 설치)")
    ap.add_argument("--arduino-cli", help="사용할 arduino-cli 실행 파일 경로 (기본: 자동 탐색/설치)")
    ap.add_argument("--build-dir", default=BUILD_DIR, help=f"빌드 캐시 폴더 (기본: {BUILD_DIR})")
    return ap.parse_args()


def main():
    args = parse_args()
    start = time.time()

    if args.list_ports:
        ports = list_ports()
        if not ports:
            print("ESP32 후보 포트가 없습니다. (USB 케이블/전원 확인)")
        else:
            print("ESP32 후보 포트:")
            for p in ports:
                print("  " + describe_port(p))
        return 0

    if not os.path.isfile(SKETCH_FILE):
        raise Fail(f"스케치를 찾을 수 없습니다: {SKETCH_FILE}\n"
                   "  (아두이노 규칙: .ino 파일 이름과 폴더 이름이 같아야 합니다)")
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        warn("sudo(root)로 실행 중입니다. 도구가 root 계정에 설치되므로 일반 사용자로 실행하는 것을 권장합니다.")

    port = None
    auto_reset = True
    if args.compile_only:
        _step["total"] = 3
    else:
        # 오래 걸리는 설치/컴파일 전에 보드 연결부터 확인한다
        step("ESP32 포트 찾기")
        port, auto_reset = find_port(args.port)
        if args.port:
            info(f"포트: {port}")

    step("arduino-cli 준비")
    cli = ensure_cli(args.arduino_cli)

    step("ESP32 코어 / 라이브러리 확인")
    ensure_core(cli, args.core_version, args.yes)
    ensure_libs(cli)

    step("컴파일")
    compile_sketch(cli, args.fqbn, args.build_dir, args.verbose)
    if args.compile_only:
        print(f"\n[완료] 컴파일 성공 (총 {fmt_elapsed(time.time() - start)})")
        return 0

    step("업로드")
    killed = free_port(port, args.yes)
    speeds = [args.baud] if args.baud else UPLOAD_SPEEDS
    ok = upload(cli, args.fqbn, port, args.build_dir, speeds, auto_reset, args.yes, args.verbose)
    if ok and not args.no_verify:
        try:
            verify_boot(port, auto_reset)
        except Exception as e:  # 확인 단계의 문제로 성공한 업로드를 실패로 만들지 않는다
            warn(f"부팅 확인 중 오류가 났습니다 (업로드 자체는 끝났습니다): {e}")

    if ok:
        print(f"\n[완료] 펌웨어 업로드 성공 (총 {fmt_elapsed(time.time() - start)})")
    else:
        print("\n[실패] 펌웨어를 업로드하지 못했습니다. 위 메시지를 확인하세요.")
    if killed:
        print("업로드를 위해 종료한 프로그램은 다시 실행해 주세요:")
        for _, cmd in killed:
            print("    " + cmd)
    return 0 if ok else 1


if __name__ == "__main__":
    # 콘솔 인코딩이 표현하지 못하는 문자가 섞여도 죽지 않도록
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n[실패] {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n중단했습니다.")
        sys.exit(130)
