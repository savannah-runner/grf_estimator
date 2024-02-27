* log2raisim_transformer.m을 실행하여 log파일을 main.cpp가 읽어들일 generalized_position.txt, generalized_velocity.txt, generalized_acceleration.txt, generalized_torque.txt (총 4개 .txt파일)을 생성한다.

    5열의 filepath = ""에 원하는 log의 주소를 입력한다.



* main.cpp를 실행하여 해당 log를 replay하고, 계산한 ground reaction force를 각 발마다 grf_RR.csv, grf_RL.csv, grf_FR.csv, grf_FL.csv (총 4개 .csv파일)을 생성한다.
  
    35, 45, 55, 65행에서 읽어들일 generalized_x.txt파일의 주소를 지정한다.
  
    234행의 //std::this_thread::sleep_for(std::chrono::microseconds (500));의 주석을 풀어 replay영상을 Unity로 볼 수 있다.
  
    79, 80행에서 Hound 1 혹은 2의 urdf중 무엇을 사용할 지 고를 수 있다.
  
    78행에서 urdf파일의 위치를 지정한다.
  
    146~149행에서 grf_XX.csv파일을 저장할 위치를 지정한다.



* grf_viewer.m을 실행하여 grf_XX.csv 파일을 plot한다.
  
    1행의 path = ""에 확인하고자 하는 .csv파일의 주소를 입력한다.
  
    14행의 f_cut은 Fourier transform에서 cut-off frequency 값으로, 2차 차분으로 인한 노이즈를 제거한다.
