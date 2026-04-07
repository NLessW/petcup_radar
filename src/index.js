// DFRobot SEN0395 mmWave 레이더 센서 - Web Serial API
class RadarSensor {
    constructor() {
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.connected = false;
        this.packetCount = 0;
        this.canvas = document.getElementById('radarCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.detectionHistory = [];
        this.maxHistory = 50;
        this.readBuffer = new Uint8Array();
        this.polling = false;

        this.initUI();
        this.startAnimation();
    }

    initUI() {
        document
            .getElementById('connectBtn')
            .addEventListener('click', () => this.connect());
        document
            .getElementById('disconnectBtn')
            .addEventListener('click', () => this.disconnect());
    }

    log(message, type = 'info') {
        const logContainer = document.getElementById('logContainer');
        const entry = document.createElement('div');
        entry.className = `log-entry ${type}`;
        const timestamp = new Date().toLocaleTimeString('ko-KR');
        entry.textContent = `[${timestamp}] ${message}`;
        logContainer.appendChild(entry);
        logContainer.scrollTop = logContainer.scrollHeight;

        // 로그가 너무 많으면 제거
        if (logContainer.children.length > 100) {
            logContainer.removeChild(logContainer.firstChild);
        }
    }

    async connect() {
        try {
            // Web Serial API 지원 확인
            if (!('serial' in navigator)) {
                alert(
                    '이 브라우저는 Web Serial API를 지원하지 않습니다. Chrome 또는 Edge를 사용해주세요.',
                );
                return;
            }

            this.log('시리얼 포트 선택 중...', 'info');

            // 시리얼 포트 요청
            this.port = await navigator.serial.requestPort();

            // 선택된 보드레이트 가져오기
            const baudRate = parseInt(
                document.getElementById('baudRate').value,
            );
            this.log(`보드레이트: ${baudRate} 사용`, 'info');

            // 포트 열기 (CDJQ314 센서의 일반적인 설정)
            await this.port.open({
                baudRate: baudRate,
                dataBits: 8,
                stopBits: 1,
                parity: 'none',
                flowControl: 'none',
            });

            this.connected = true;
            this.log('COM 포트에 연결되었습니다!', 'success');
            this.updateStatus(true);

            // 데이터 읽기 및 폴링 시작
            this.startReading();
            this.startPolling();
        } catch (error) {
            this.log(`연결 오류: ${error.message}`, 'error');
            console.error('연결 오류:', error);
        }
    }

    // CRC16 계산 (Modbus RTU)
    calculateCRC16(buffer) {
        let crc = 0xffff;
        for (let pos = 0; pos < buffer.length; pos++) {
            crc ^= buffer[pos];
            for (let i = 8; i !== 0; i--) {
                if ((crc & 0x0001) !== 0) {
                    crc >>= 1;
                    crc ^= 0xa001;
                } else {
                    crc >>= 1;
                }
            }
        }
        // 바이트 스왑
        return ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    }

    // 센서에 거리 요청 명령 전송
    async sendDistanceCommand() {
        try {
            // 명령: 0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36
            const command = new Uint8Array([
                0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36,
            ]);

            const writer = this.port.writable.getWriter();
            await writer.write(command);
            writer.releaseLock();

            this.log(
                `명령 전송: ${Array.from(command)
                    .map((b) => '0x' + b.toString(16).padStart(2, '0'))
                    .join(' ')}`,
                'info',
            );
        } catch (error) {
            this.log(`명령 전송 오류: ${error.message}`, 'error');
        }
    }

    // 주기적으로 센서에 명령 전송 (500ms 간격)
    async startPolling() {
        this.polling = true;

        while (this.polling && this.connected) {
            await this.sendDistanceCommand();
            await new Promise((resolve) => setTimeout(resolve, 500));
        }
    }

    async initializeSensor() {
        // Modbus 방식에서는 초기화 불필요
        this.log('센서 준비 완료', 'success');
    }

    async startReading() {
        try {
            this.reader = this.port.readable.getReader();

            this.log('데이터 수신 대기 중...', 'info');

            while (true) {
                const { value, done } = await this.reader.read();
                if (done) {
                    this.log('Reader 종료됨', 'info');
                    break;
                }

                if (value) {
                    // Raw 바이트 데이터 로깅
                    this.log(
                        `수신 (${value.length} bytes): ${Array.from(value)
                            .map((b) => '0x' + b.toString(16).padStart(2, '0'))
                            .join(' ')}`,
                        'success',
                    );

                    // 버퍼에 추가
                    const newBuffer = new Uint8Array(
                        this.readBuffer.length + value.length,
                    );
                    newBuffer.set(this.readBuffer);
                    newBuffer.set(value, this.readBuffer.length);
                    this.readBuffer = newBuffer;

                    // Modbus 응답 파싱 시도
                    this.parseModbusResponse();
                }
            }
        } catch (error) {
            if (this.connected) {
                this.log(`읽기 오류: ${error.message}`, 'error');
                console.error('읽기 오류:', error);
            }
        } finally {
            if (this.reader) {
                this.reader.releaseLock();
            }
        }
    }

    parseModbusResponse() {
        // Modbus 응답 형식: 0x01 0x03 0x02 [DATA_H] [DATA_L] [CRC_L] [CRC_H]
        // 최소 7바이트 필요
        while (this.readBuffer.length >= 7) {
            // 헤더 찾기: 0x01 0x03
            let headerIndex = -1;
            for (let i = 0; i < this.readBuffer.length - 1; i++) {
                if (
                    this.readBuffer[i] === 0x01 &&
                    this.readBuffer[i + 1] === 0x03
                ) {
                    headerIndex = i;
                    break;
                }
            }

            if (headerIndex === -1) {
                // 헤더 없음 - 버퍼 비우기
                this.readBuffer = new Uint8Array();
                break;
            }

            // 헤더 이전 데이터 제거
            if (headerIndex > 0) {
                this.readBuffer = this.readBuffer.slice(headerIndex);
            }

            // 충분한 데이터가 있는지 확인
            if (this.readBuffer.length < 7) {
                break;
            }

            // 데이터 길이 확인
            if (this.readBuffer[2] === 0x02) {
                // 패킷 추출
                const packet = this.readBuffer.slice(0, 7);

                // CRC 검증
                const dataPart = packet.slice(0, 5);
                const receivedCRC = (packet[5] << 8) | packet[6];
                const calculatedCRC = this.calculateCRC16(dataPart);

                this.log(
                    `CRC 검증: 수신=${receivedCRC.toString(16)}, 계산=${calculatedCRC.toString(16)}`,
                    'info',
                );

                if (receivedCRC === calculatedCRC) {
                    // 거리 데이터 추출 (mm 단위)
                    const distance = (packet[3] << 8) | packet[4];

                    this.packetCount++;
                    document.getElementById('packetCount').textContent =
                        this.packetCount;

                    this.log(`거리 측정: ${distance} mm`, 'success');

                    // cm로 변환하여 표시
                    const distanceCm = distance / 10;
                    const detected = distance > 0 && distance < 5000; // 5m 이내
                    this.updateDisplay(distanceCm, null, detected);
                } else {
                    this.log('CRC 오류!', 'error');
                }

                // 처리된 패킷 제거
                this.readBuffer = this.readBuffer.slice(7);
            } else {
                // 잘못된 패킷 - 1바이트 건너뛰기
                this.readBuffer = this.readBuffer.slice(1);
            }
        }
    }

    updateDisplay(distance, strength, detected) {
        // 거리 표시
        if (distance !== null && distance !== undefined) {
            document.getElementById('distance').textContent =
                distance.toFixed(1);
        }

        // 신호 강도 표시
        if (strength !== null && strength !== undefined) {
            document.getElementById('strength').textContent =
                strength.toFixed(0);
        }

        // 감지 상태
        const detectionEl = document.getElementById('detection');
        if (detected) {
            detectionEl.textContent = '감지됨';
            detectionEl.style.color = '#38a169';
        } else {
            detectionEl.textContent = '없음';
            detectionEl.style.color = '#718096';
        }

        // 히스토리에 추가
        this.detectionHistory.push({
            distance: distance || 0,
            strength: strength || 0,
            detected: detected,
            timestamp: Date.now(),
        });

        if (this.detectionHistory.length > this.maxHistory) {
            this.detectionHistory.shift();
        }
    }

    startAnimation() {
        const animate = () => {
            this.drawRadar();
            requestAnimationFrame(animate);
        };
        animate();
    }

    drawRadar() {
        const { width, height } = this.canvas;
        const centerX = width / 2;
        const centerY = height - 50;
        const maxRadius = Math.min(width, height) * 0.8;

        // 배경 클리어
        this.ctx.fillStyle = '#1a202c';
        this.ctx.fillRect(0, 0, width, height);

        // 그리드 원 그리기
        this.ctx.strokeStyle = '#2d3748';
        this.ctx.lineWidth = 1;

        for (let i = 1; i <= 4; i++) {
            const radius = (maxRadius / 4) * i;
            this.ctx.beginPath();
            this.ctx.arc(centerX, centerY, radius, Math.PI, 0);
            this.ctx.stroke();

            // 거리 라벨
            this.ctx.fillStyle = '#4a5568';
            this.ctx.font = '12px Arial';
            this.ctx.fillText(
                `${i * 50}cm`,
                centerX + radius - 30,
                centerY - 5,
            );
        }

        // 중심선들
        const angles = [-90, -60, -30, 0, 30, 60, 90];
        angles.forEach((angle) => {
            const rad = (angle * Math.PI) / 180;
            this.ctx.beginPath();
            this.ctx.moveTo(centerX, centerY);
            this.ctx.lineTo(
                centerX + Math.cos(rad) * maxRadius,
                centerY + Math.sin(rad) * maxRadius,
            );
            this.ctx.stroke();
        });

        // 감지된 객체 표시
        if (this.detectionHistory.length > 0) {
            const latest =
                this.detectionHistory[this.detectionHistory.length - 1];

            if (latest.detected && latest.distance > 0) {
                const distance = Math.min(latest.distance, 200);
                const radius = (distance / 200) * maxRadius;

                // 펄스 효과
                const time = Date.now() / 1000;
                const pulse = Math.sin(time * 5) * 0.2 + 0.8;

                // 객체 표시
                this.ctx.fillStyle = `rgba(72, 187, 120, ${pulse})`;
                this.ctx.beginPath();
                this.ctx.arc(centerX, centerY - radius, 10, 0, Math.PI * 2);
                this.ctx.fill();

                // 원형 웨이브
                this.ctx.strokeStyle = `rgba(72, 187, 120, ${0.5 * pulse})`;
                this.ctx.lineWidth = 2;
                this.ctx.beginPath();
                this.ctx.arc(
                    centerX,
                    centerY - radius,
                    15 + pulse * 10,
                    0,
                    Math.PI * 2,
                );
                this.ctx.stroke();
            }
        }

        // 스캔 라인 애니메이션
        const scanAngle = ((Date.now() / 1000) % 2) * Math.PI - Math.PI;
        const gradient = this.ctx.createLinearGradient(
            centerX,
            centerY,
            centerX + Math.cos(scanAngle) * maxRadius,
            centerY + Math.sin(scanAngle) * maxRadius,
        );
        gradient.addColorStop(0, 'rgba(102, 126, 234, 0.8)');
        gradient.addColorStop(1, 'rgba(102, 126, 234, 0)');

        this.ctx.strokeStyle = gradient;
        this.ctx.lineWidth = 3;
        this.ctx.beginPath();
        this.ctx.moveTo(centerX, centerY);
        this.ctx.lineTo(
            centerX + Math.cos(scanAngle) * maxRadius,
            centerY + Math.sin(scanAngle) * maxRadius,
        );
        this.ctx.stroke();

        // 히스토리 그래프 (하단)
        if (this.detectionHistory.length > 1) {
            this.ctx.strokeStyle = '#48bb78';
            this.ctx.lineWidth = 2;
            this.ctx.beginPath();

            const graphY = height - 20;
            const graphWidth = width - 40;
            const step = graphWidth / this.maxHistory;

            this.detectionHistory.forEach((point, index) => {
                const x = 20 + index * step;
                const y = graphY - (point.distance / 200) * 30;

                if (index === 0) {
                    this.ctx.moveTo(x, y);
                } else {
                    this.ctx.lineTo(x, y);
                }
            });

            this.ctx.stroke();
        }
    }

    updateStatus(connected) {
        const statusEl = document.getElementById('status');
        const connectBtn = document.getElementById('connectBtn');
        const disconnectBtn = document.getElementById('disconnectBtn');

        if (connected) {
            statusEl.textContent = '연결됨 - 데이터 수신 중';
            statusEl.className = 'status connected';
            connectBtn.disabled = true;
            disconnectBtn.disabled = false;
        } else {
            statusEl.textContent = '연결되지 않음';
            statusEl.className = 'status disconnected';
            connectBtn.disabled = false;
            disconnectBtn.disabled = true;
        }
    }

    async disconnect() {
        try {
            this.connected = false;
            this.polling = false;

            if (this.reader) {
                await this.reader.cancel();
                this.reader.releaseLock();
                this.reader = null;
            }

            if (this.writer) {
                this.writer.releaseLock();
                this.writer = null;
            }

            if (this.port) {
                await this.port.close();
                this.port = null;
            }

            this.log('연결이 해제되었습니다.', 'info');
            this.updateStatus(false);
        } catch (error) {
            this.log(`연결 해제 오류: ${error.message}`, 'error');
            console.error('연결 해제 오류:', error);
        }
    }
}

// 앱 초기화
document.addEventListener('DOMContentLoaded', () => {
    new RadarSensor();
});
