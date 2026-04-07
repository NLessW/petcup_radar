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

            // 데이터 읽기 시작
            this.startReading();
        } catch (error) {
            this.log(`연결 오류: ${error.message}`, 'error');
            console.error('연결 오류:', error);
        }
    }

    async startReading() {
        try {
            this.reader = this.port.readable.getReader();
            let buffer = new Uint8Array();
            let textBuffer = '';

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
                        `수신 (${value.length} bytes): ${Array.from(
                            value.slice(0, 20),
                        )
                            .map((b) => b.toString(16).padStart(2, '0'))
                            .join(' ')}`,
                        'success',
                    );

                    // 텍스트로 디코딩 시도
                    try {
                        const text = new TextDecoder().decode(value);
                        textBuffer += text;

                        // 줄바꿈으로 데이터 분리
                        const lines = textBuffer.split(/[\r\n]+/);
                        textBuffer = lines.pop(); // 마지막 불완전한 라인은 버퍼에 유지

                        for (const line of lines) {
                            if (line.trim()) {
                                this.processData(line.trim());
                            }
                        }
                    } catch (e) {
                        // 텍스트 디코딩 실패 - 바이너리 데이터
                        this.processBinaryData(value);
                    }
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

    processBinaryData(data) {
        this.packetCount++;
        document.getElementById('packetCount').textContent = this.packetCount;

        // 바이너리 데이터 파싱 (SEN0395는 일반적으로 텍스트 출력)
        this.log(
            `바이너리 (${data.length} bytes): ${Array.from(data)
                .map((b) => b.toString(16).padStart(2, '0'))
                .join(' ')}`,
            'info',
        );

        // SEN0395 바이너리 프로토콜 (제조사 문서 참조)
        // 일반적인 패킷 구조: [Header][Length][Data][Checksum]
        if (data.length >= 8) {
            // 헤더 확인 (예: 0xAA 0x55)
            if (data[0] === 0xaa && data[1] === 0x55) {
                const length = data[2];
                const cmd = data[3];

                // 거리 데이터 명령 (예: 0x03)
                if (cmd === 0x03 && data.length >= length) {
                    const distance = data[4] | (data[5] << 8);
                    const detected = data[6] === 0x01;
                    this.updateDisplay(distance, null, detected);
                    return;
                }
            }
        }

        // 폴백: 첫 2바이트를 거리로 간주 (리틀 엔디안)
        if (data.length >= 2) {
            const distance = data[0] | (data[1] << 8);
            if (distance > 0 && distance < 10000) {
                this.updateDisplay(distance / 10, null, distance < 2000);
            }
        }
    }

    processData(data) {
        this.packetCount++;
        document.getElementById('packetCount').textContent = this.packetCount;

        // Raw 데이터 로깅
        this.log(
            `텍스트: ${data.substring(0, 100)}${data.length > 100 ? '...' : ''}`,
            'info',
        );

        // DFRobot SEN0395 센서 데이터 파싱
        // 출력 형식: "$JYBSS,0,target,123,0,0,0*XX" 또는 간단한 텍스트

        // SEN0395 형식: $JYBSS 프로토콜
        if (data.startsWith('$JYBSS')) {
            const parts = data.split(',');
            if (parts.length >= 4) {
                const status = parts[1]; // 0=no target, 1=target
                const type = parts[2]; // 'target' or 'noTarget'
                const distance = parseInt(parts[3]); // distance in cm

                const detected = status === '1' || type === 'target';
                this.updateDisplay(distance, null, detected);
                return;
            }
        }

        // 간단한 출력 형식: "Target detected at 123cm"
        if (data.includes('Target') || data.includes('target')) {
            const distMatch = data.match(/(\d+)\s*cm/i);
            if (distMatch) {
                const distance = parseInt(distMatch[1]);
                this.updateDisplay(distance, null, true);
                return;
            }
        }

        // "No target" 또는 "noTarget"
        if (
            data.toLowerCase().includes('no target') ||
            data.toLowerCase().includes('notarget')
        ) {
            this.updateDisplay(0, null, false);
            return;
        }

        // 범용 형식 1: JSON 형태
        try {
            const jsonData = JSON.parse(data);
            this.updateDisplay(
                jsonData.distance,
                jsonData.strength,
                jsonData.detected,
            );
            return;
        } catch (e) {
            // JSON이 아님
        }

        // 범용 형식 2: "거리:100,강도:50" 형태
        const distanceMatch =
            data.match(/거리[:=\s]*(\d+\.?\d*)/i) ||
            data.match(/distance[:=\s]*(\d+\.?\d*)/i) ||
            data.match(/dist[:=\s]*(\d+\.?\d*)/i);

        const strengthMatch =
            data.match(/강도[:=\s]*(\d+\.?\d*)/i) ||
            data.match(/strength[:=\s]*(\d+\.?\d*)/i) ||
            data.match(/rssi[:=\s]*(-?\d+\.?\d*)/i);

        if (distanceMatch) {
            const distance = parseFloat(distanceMatch[1]);
            const strength = strengthMatch
                ? parseFloat(strengthMatch[1])
                : null;
            const detected = distance < 200; // 2m 이내면 감지로 판단

            this.updateDisplay(distance, strength, detected);
            return;
        }

        // 형식 3: 숫자만 있는 경우 (거리 값으로 간주)
        const numericMatch = data.match(/(\d+\.?\d*)/);
        if (numericMatch) {
            const distance = parseFloat(numericMatch[1]);
            this.updateDisplay(distance, null, distance < 200);
            return;
        }

        // 형식 4: 16진수 데이터
        if (data.match(/^[0-9A-Fa-f\s]+$/)) {
            this.log(`HEX 데이터: ${data}`, 'info');
            // 16진수 파싱 로직은 센서 프로토콜에 따라 구현
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

            if (this.reader) {
                await this.reader.cancel();
                this.reader.releaseLock();
                this.reader = null;
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
