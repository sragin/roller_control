import paho.mqtt.client as mqtt
import time

# MQTT 브로커에 연결되었을 때 호출되는 콜백 함수
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully.")
        # 토픽 구독
        client.subscribe("command/workplan/roller")
    else:
        print(f"Connection failed with code {rc}")

# 메시지를 받을 때 호출되는 콜백 함수
def on_message(client, userdata, msg):
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")

# 연결이 끊겼을 때 호출되는 콜백 함수
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection. Trying to reconnect...")
    else:
        print("Disconnected successfully.")

    # 재연결 로직
    while True:
        try:
            client.reconnect()  # 재연결 시도
            print("Reconnected successfully.")
            break
        except Exception as e:
            print(f"Reconnect failed: {e}")
            time.sleep(5)  # 재연결 시도 전 대기 시간 (5초)

# MQTT 클라이언트 생성
client = mqtt.Client()
# MQTT 클라이언트 생성 (클라이언트 ID 추가)
client_id = "roller_cmd_recv_vib"
client = mqtt.Client(client_id)

# 유저네임과 비밀번호 설정
client.username_pw_set("roller", "roller")

# 콜백 함수 등록
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

# MQTT 브로커에 연결 (브로커 주소와 포트 번호 필요)
client.connect("withpoints.asuscomm.com", 50592, 60)

# 메시지 루프 시작
client.loop_start()  # 비동기 메시지 루프를 시작하여 메시지를 계속 받음

# 스크립트가 계속 실행되도록 무한 대기
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
    client.disconnect()  # 종료 시 연결 해제
    client.loop_stop()  # 메시지 루프 종료
