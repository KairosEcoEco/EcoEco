# arduino
아두이노로 쓰레기가 컨베이어 벨트 위로 동시에 들어오는 것을 방지하기 위한 개폐장치 제작했습니다. 

### 역할
- **서보 모터**: 컨베이어 벨트로 가는 입구를 열고 닫습니다.    
- **IR 센서**: 컨베이어 벨트로 들어가는 쓰레기를 감지합니다. 
- **리셋 버튼**: 프로그램을 리셋합니다.

### 기능
- IR 센서가 컨베이어로 들어가는 1번째 쓰레기를 감지하면 바로 서보 모터가 작동하고 2번째 쓰레기부터는 0.5초 이상 감지되야 서보 모터가 작동합니다.  
- 작동한 서보 모터는 2초 후에 원상태로 복귀합니다.
