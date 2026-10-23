echo "=== My App Boot Script ==="
echo "Time: $(date)"
echo

cd /home/pi/Desktop/web_service || {
    echo "프로젝트 폴더로 이동 실패"
    read -p "Enter를 누르면 닫힘..."
    exit 1
}

# Flask는 0.0.0.0으로 열리므로 인터넷 없이도 실행됨
# 핫스팟 IP를 받을 때까지만 최대 30초 대기 (못 받아도 그냥 실행)
echo "네트워크(IP) 확인 중..."
for i in $(seq 1 15); do
    if hostname -I | grep -q "192.168.137."; then
        echo "IP 확인됨: $(hostname -I)"
        break
    fi
    echo "아직 IP 없음... 2초 후 재시도 ($i/15)"
    sleep 2
done

echo "5초간 대기 중..."
sleep 5
echo


echo "Python 코드 실행 시작"
echo "--------------------------------"

python3 -u flask_web.py

echo "--------------------------------"
echo "Python 코드가 종료됨. 종료 코드: $?"
echo

read -p "터미널을 닫으려면 Enter를 누르세요..."