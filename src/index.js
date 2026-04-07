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

    // 주기적으로 센서에 명령 전송 (100ms 간격)
    async startPolling() {
        this.polling = true;

        while (this.polling && this.connected) {
            await this.sendDistanceCommand();
            await new Promise((resolve) => setTimeout(resolve, 200));
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
        // 거리 표시 (mm 단위)
        if (distance !== null && distance !== undefined) {
            const distanceMm = distance * 10; // cm를 mm로 변환
            const distanceElement = document.getElementById('distance');
            if (distanceElement) {
                distanceElement.textContent = distanceMm.toFixed(0);
            }
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
            this.drawContainer();
            requestAnimationFrame(animate);
        };
        animate();
    }

    drawContainer() {
        const { width, height } = this.canvas;
        const padding = 40;
        const containerWidth = width - padding * 2;
        const containerHeight = height - padding * 2;
        const containerX = padding;
        const containerY = padding;

        // 배경 클리어
        this.ctx.fillStyle = '#1a202c';
        this.ctx.fillRect(0, 0, width, height);

        // 컨테이너 외곽선
        this.ctx.strokeStyle = '#4a5568';
        this.ctx.lineWidth = 3;
        this.ctx.strokeRect(
            containerX,
            containerY,
            containerWidth,
            containerHeight,
        );

        // 측정 최대 거리 (mm 단위, 예: 575mm)
        const maxDistance = 575;

        // 현재 거리 가져오기
        let currentDistance = 0;
        if (this.detectionHistory.length > 0) {
            const latest =
                this.detectionHistory[this.detectionHistory.length - 1];
            // cm를 mm로 변환
            currentDistance = latest.distance * 10;
            // 최대값 제한
            if (currentDistance > maxDistance) {
                currentDistance = maxDistance;
            }
        }

        // 채움 비율 계산 (거리가 가까울수록 많이 채움)
        let fillRatio = 0;
        if (currentDistance >= 0 && currentDistance <= maxDistance) {
            fillRatio = 1 - currentDistance / maxDistance;
        }

        // 채워지는 높이 계산
        const fillHeight = containerHeight * fillRatio;

        // 그라데이션 색상 (채워진 정도에 따라)
        const gradient = this.ctx.createLinearGradient(
            containerX,
            containerY + containerHeight,
            containerX,
            containerY,
        );

        if (fillRatio > 0.8) {
            // 거의 가득 참 - 빨간색
            gradient.addColorStop(0, '#f56565');
            gradient.addColorStop(1, '#fc8181');
        } else if (fillRatio > 0.5) {
            // 반 이상 - 주황색
            gradient.addColorStop(0, '#ed8936');
            gradient.addColorStop(1, '#f6ad55');
        } else if (fillRatio > 0.2) {
            // 조금 찬 상태 - 노란색
            gradient.addColorStop(0, '#ecc94b');
            gradient.addColorStop(1, '#f6e05e');
        } else if (fillRatio > 0) {
            // 거의 비어있음 - 초록색
            gradient.addColorStop(0, '#48bb78');
            gradient.addColorStop(1, '#68d391');
        }

        // 채워진 영역 그리기
        if (fillHeight > 0) {
            this.ctx.fillStyle = gradient;
            this.ctx.fillRect(
                containerX,
                containerY + containerHeight - fillHeight,
                containerWidth,
                fillHeight,
            );

            // 물결 효과
            const time = Date.now() / 1000;
            const waveAmplitude = 5;
            const waveFrequency = 2;
            const waveY = containerY + containerHeight - fillHeight;

            this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
            this.ctx.lineWidth = 2;
            this.ctx.beginPath();

            for (let x = 0; x <= containerWidth; x += 5) {
                const y =
                    waveY +
                    Math.sin(x / 50 + time * waveFrequency) * waveAmplitude;
                if (x === 0) {
                    this.ctx.moveTo(containerX + x, y);
                } else {
                    this.ctx.lineTo(containerX + x, y);
                }
            }
            this.ctx.stroke();
        }

        // 눈금선 그리기
        this.ctx.strokeStyle = '#2d3748';
        this.ctx.lineWidth = 1;
        this.ctx.setLineDash([5, 5]);

        for (let i = 1; i <= 4; i++) {
            const y = containerY + (containerHeight / 4) * i;
            this.ctx.beginPath();
            this.ctx.moveTo(containerX, y);
            this.ctx.lineTo(containerX + containerWidth, y);
            this.ctx.stroke();

            // 거리 라벨
            const distLabel = maxDistance - (maxDistance / 4) * i;
            this.ctx.fillStyle = '#a0aec0';
            this.ctx.font = '14px Arial';
            this.ctx.setLineDash([]);
            this.ctx.fillText(`${distLabel}mm`, containerX - 50, y + 5);
        }

        this.ctx.setLineDash([]);

        // 중앙에 현재 거리 표시
        this.ctx.fillStyle = '#e2e8f0';
        this.ctx.font = 'bold 48px Arial';
        this.ctx.textAlign = 'center';
        this.ctx.fillText(
            `${currentDistance.toFixed(0)} mm`,
            containerX + containerWidth / 2,
            containerY + containerHeight / 2,
        );

        // 퍼센트 표시
        this.ctx.font = 'bold 24px Arial';
        this.ctx.fillStyle = '#cbd5e0';
        this.ctx.fillText(
            `${(fillRatio * 100).toFixed(1)}% 채움`,
            containerX + containerWidth / 2,
            containerY + containerHeight / 2 + 40,
        );

        this.ctx.textAlign = 'left';
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
