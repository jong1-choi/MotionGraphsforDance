# Motion graphs for dance
아주대 미디어학과 석사학위 졸업 논문 '춤 동작을 위한 모션 그래프 생성 및 음악 동기화' 구현체

결과 링크
https://youtu.be/gxfe8FVCXWY?si=cXpu5NE6Ko3vAxDA

## 논문 소개

<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image1.png">
먼저 본 논문의 목적인 춤 동작 데이터 생성의 필요성에 대해 이야기 하겠습니다. 3D 콘텐츠 분야에서 캐릭터의 움직임을 위해 모션 데이터는 반드시 필요합니다. 게임 분야 및 메타버스의 꾸준한 성장으로 이러한 수요는 증가하고 있으며, 특히 메타버스에서 유저 기반의 콘텐츠 생성으로 인해 ‘개인’의 데이터 수요가 증가 하였습니다. 하지만 고품질의 모션 데이터를 얻기 위해서는 3D 애니메이터들의 제작 또는 모션 캡쳐 과정이 필요합니다. 이는 전문적인 인력이 필요로 하며 상당한 비용과 시간이 발생하기 때문에 일반적인 사용자가 이를 통해 모션 데이터를 얻기는 사실상 불가능 합니다. 일반적인 동작들은 기존 합성기술을 이용하여 어느정도 생성이 가능하지만, 음악에 맞춰 여러 동작이 조합되어 있는 춤 데이터는 이러한 방법으로 생성이 어렵습니다. 따라서 본 논문에서 적은 데이터로도 사용자들이 간편하게 춤 동작 데이터를 생성할 수 있는 방법에 대해 소개합니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image2.png">
본 논문의 방법들에 사용된 관련 연구들에 대해 이야기 하겠습니다. 먼저 모션 데이터를 마디 단위로 잘라 합성하는 본 논문의 방법은 ‘choreo master’에서 영감을 받아 사용하였고, 마디 단위의 동작들을 그래프의 노드로 만들고 연결 될 수 있는 마디들을 엣지로 연결하여 탐색하기 위해 ‘motion graphs’ 논문의 구조를 사용했습니다. 이렇게 모션 그래프를 생성하기 위해선 비트를 찾고 이를 통해 마디를 추출해야 합니다. 이를 위해 ‘Extraction and alignment evaluation of motion beats for street dance’의 방법을 사용하여 관절들의 속도 크기를 계산하고 속도가 0에 가까워 지면서 극솟값에 해당하는 지점을 후보 비트로 선정하였습니다. 이렇게 검출된 후보 비트에 본 논문에서 제시하는 조건을 적용시켜 정제된 비트를 구하고 이를 기반으로 마디를 추출합니다. 마디를 추출하기 위해 ‘Rhythmic-motion synthesis based on motion-beat analysis’에서 비트들에 대해 FFT를 진행하고, 도미넌트 프리퀀시를 찾아 주기를 구하는 방법을 적용했습니다. 마지막으로 음악과 길이를 맞추기 위한 타임워핑, 모션들의 회전과 위치를 블렌딩하기 위한 방법은 ‘On-line locomotion generation based on motion blending’에서 소개한 방법을 사용하였습니다. 
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image3.png">
다음은 앞서 소개한 방법에 대한 본 논문의 개요입니다.우선 기존의 춤 동작 데이터에서 최대한 많은 동작들을 나누어 조합하기 위해, 동작을 나타내는 최소 단위인 마디를 추출합니다. 이를 위해서는 먼저 춤 동작의 비트를 찾아야 합니다. 비트를 찾고 난 후 FFT를 통해 비트들의 dominant frequancy를 구해 평균 주기를 추출합니다. 이렇게 추출된 주기와 유사한 간격을 갖는 비트들을 묶어 마디를 만듭니다. 이제 마디 사이의 유사도를 구해 모션 그래프를 생성합니다. 이때 마디는 그래프의 노드가 되고, 연결될 수 있는 마디들을 엣지로 연결합니다. 하지만 마디들은 각 주기에 맞춰 생성되었기 때문에 길이가 다릅니다. 음악과 동기화를 위해서 춤 동작의 마디 길이도 음악의 마디와 맞춰야 하기 때문에 입력된 음악 정보에 맞춰 타임 워핑을 진행합니다. 마지막으로 그래프를 탐색하여 음악에 배치될 마디를 찾고, 모션 비트의 특징을 잃지 않는 방법으로 트랜지션 모션을 만들어 합성하면 새로운 춤 동작이 생성됩니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image4.png">
지금부터 비트 탐색 방법에 대하여 이야기 하겠습니다. 본 논문에서는 후보 비트를 먼저 구하고, 노이즈에 해당하는 비트들을 걸러내는 과정으로 최종 비트를 추출합니다. 이를 위해 후보 비트를 구해야 하는데 ‘Extraction and alignment evaluation of motion beats for street dance’에서 관절들의 속도 그래프로부터 극솟값에 해당하는 부분을 비트로 정의하였습니다. 본 논문에도 해당 방법을 사용하여 그림과 같이 원형으로 표시된 후보 비트를 산출하였습니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image5.png">
하지만 후보 비트들은 사람의 움직임에서 흔들림으로 인해 발생하는 노이즈들도 포함이 되어 있습니다. 위 그림에서 빨간색과 초록색으로 표시된 원들이 노이즈에 해당합니다. 빨간색 노이즈는 준비 동작이나 멈춰있는 상황같이 동작의 속도가 느릴 때 발생하고, 초록색 노이즈는 동작을 진행하는 중간에 발생합니다. 다음 영상을 통해 노이즈를 확인 할 수 있습니다. 이러한 노이즈들을 제거하기 위해 본 논문에서는 3가지 조건을 추가하여 정제된 비트를 검출합니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image6.png">
본 논문에서 제안하는 3가지 조건은 다음과 같습니다. 먼저 F1은 비트가 발생할 수 있는 최소 간격인 윈도우 크기를 기준으로, 앞 뒤 윈도우 크기 만큼의 프레임을 확인하여, 해당 프레임의 속도의 크기가 가장 작은 경우에만 비트로 검출합니다. 이 때 윈도우의 크기는 댄스 음악의 평균 bpm이 128이라는 것을 기준으로 3분의 1초에 해당하는 프레임으로 설정하였습니다. F2는 주변 프레임과 비교하여 운동량이 임계값 보다 작은 경우를 검출합니다. 이 때 춤 동작마다 속도와 동작의 크기가 다르기 때문에, 춤 동작에서 모든 프레임의 속도의 크기를 더하고 평균을 구해 평균 운동량을 임계값으로 설정합니다. 따라서 F2는 윈도우 크기만큼의 주변 프레임과 속도 크기의 차이를 구해 평균 운동량으로 작으면 비트에서 제외합니다. F1과 F2 두 조건으로 빨간색 노이즈와 대부분의 초록색 노이즈는 제거가 가능합니다. 하지만 빨간색 노이즈 보다 비교적 큰 운동량을 갖는 초록색 노이즈의 특성상 제거 되지 않는 경우가 발생합니다. 따라서 극솟값에 해당하는 부분의 속도 크기가 0에 가까울수록 강조되는 비트라는 점을 기반으로 F3 조건을 추가하였습니다. F3은 F2와 마찬가지로 평균 운동량을 사용하여, 후보 비트에 해당하는 프레임에서 속도의 크기가 평균 운동량 보다 클 경우 비트에서 제외합니다. 이렇게 3가지 조건을 사용하여 파란색으로 표시되어 있는 최종 비트를 추출합니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image7.png">
이렇게 구한 최종 비트들을 통해 마디를 추출합니다. 춤 동작을 최소 단위로 나타내기 위해 균일한 박자를 갖는 비트들을 모아 마디로 만들어야 합니다. 먼저 동작 데이터에서 비트들의 도미넌트한 주기를 찾아야 하는데  “Rhythmic-motion synthesis based on motion-beat analysis”에서 소개한 레퍼런스 비트를 구하는 방법을 사용해 마디를 추출합니다. 해당 방법은 먼저 각 비트들에 대해 코사인 형태의 sinosoidal fitting을 합니다. 이는 그림 (a)의 초록색 그래프의 형태로 나타나며, 이에 대해 FFT를 적용합니다. FFT를 통해 나온 주파수 도메인의 신호를 제곱하여 power spectral density의 형태로 나타내고, 진폭이 가장 큰 주파수를 찾아 평균 주기를 계산합니다. 주파수 도메인의 PSD 그래프는 그림 (b)와 같이 나타납니다. 마지막으로 최종 비트들에 대해 평균 주기와 유사한 간격을 갖는 4개의 비트들을 마디로 추출합니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image8.png">
지금까진 설명드린 비트 및 마디 추출 방법을 통해, (a),(b) 그림과 같이 유사한 간격을 가지고 강조되는 동작들이 모션 마디 안에 포함되어 있는 것을 확인 할 수 있습니다. (a),(b) 그림은 모션 마디를 매 프레임 마다 조금씩 이동시켜 표현하였으며, 비트가 되는 관절의 궤적을 그리고 있고, 비트에 해당하는 프레임은 빨간색으로 나타냈습니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image9.png">
이렇게 추출된 마디들을 연결하기 위해 기존 모션 그래프의 방법을 사용하였습니다. 기존의 방법과 다른 점은 추출된 모션 마디를 그래프의 노드로 사용하고, 마디의 마지막 프레임과 연결되는 마디의 첫 번째 프레임을 엣지로 생성하는 것 입니다. 연결의 자연스러움을 위해 거리값이 임계값 보다 낮을 경우 엣지를 생성하며 sink,dead end를 제거하기 위해 SCC 알고리즘을 적용하여 가지치기를 진행합니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image10.png">
음악과의 동기화를 위해 먼저 마디의 구조를 맞춰야 합니다. 본 논문에서 추출한 마디는 시작과 끝 프레임이 비트의 프레임이기 때문에, 위의 그림과 같이 연속적으로 마디를 붙이기 위해서는 음악의 마디처럼 추가 프레임을 설정해야 합니다. 비트의 간격을 균일하게 만들기 위해 마디 프레임의 3분의 1만큼 추가합니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image11.png">
음악과 모션의 마디 구조를 동기화 했다면 마디의 길이도 타임 워핑을 통해 맞춰야 합니다. 각 모션 데이터는 비트들의 평균 주기가 다르기 때문에 생성되는 마디의 길이도 다릅니다. 적은 데이터로도 다양한 춤 동작을 만들기 위해, 음악과 동기화된 마디의 구조를 입력받은 BPM에 맞춰 타임 워핑 시킵니다. 예를 들어 30 FPS의 모션 데이터를 120 BPM에 맞추기 위해서는, 60초를 BPM으로 나누어 비트가 반복되는 시간을 구하고 4를 곱해 4개의 비트가 지속되는 시간을 구합니다. 이렇게 구한 시간을 프레임에 맞춰 FPS를 곱해주면 마디의 프레임 수가 계산됩니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image12.png">
음악과 동기화된 춤 동작을 생성하기 위한 준비 단계가 모두 끝나고, 탐색된 모션 마디들을 적절히 합성하면 춤 동작이 생성됩니다. 모션 마디들의 자연스러운 합성을 위해서는 비트를 유지하는 블렌딩 방법이 필요합니다. 기존의 방법은 그림 (a)와 같이 모션의 앞 뒤에 트랜지션되는 윈도우의 크기 만큼 프레임을 추가하고, 루트 관절의 위치와 관절들의 회전을 블렌딩하여 트랜지션 모션을 생성합니다. 이 방법은 일반적인 동작들을 블렌딩 할 때는 자연스러운 결과를 도출합니다. 하지만 비트와 비트 사이를 연결할 때 프레임 마다 다른 모션이 블렌딩 되기 때문에, 추가된 프레임의 동작이 큰 경우, 그림 (b)에서 볼 수 있듯이 움직임이 복잡해지고, 속도의 변화가 큰 경우가 발생합니다. 이러한 경우 트랜지션 모션에서 비트가 발생하거나, 연결된 비트가 잘 관측되지 않을 수 있습니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image13.png">
이러한 문제를 해결하기 위해 본 논문에서는 변형된 블렌딩 방법을 제시합니다. 해당 방법은 그림 (a)와 같이 앞 모션의 마지막 프레임만을 사용하고, 뒤 모션은 기존과 같이 프레임을 추가하여 블렌딩 합니다. 이런 방식을 사용할 경우 앞 모션의 마지막 비트가 더 오래 남아있기 때문에 더 강조되는 효과를 가지며, 뒤 모션은 해당 모션의 앞 부분을 추가 프레임으로 사용하기 때문에 자연스럽게 블렌딩이 가능합니다. 그림 (b)를 보면 알 수 있듯이 변환 동작의 움직임이 부드럽고, 속도의 변화가 작은 것을 확인 할 수 있습니다. 이러한 이유로 비트와 비트가 연결 될 때 변환 동작에서 불필요한 동작이 사라지고, 속도의 변화도 크지 않기 때문에 모션 비트와 유사한 움직임이 발생하지 않습니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image14.png">
본 논문의 방법을 식으로 나타내면 다음과 같습니다. 먼저 식(1)은 알파 블렌딩을 위해 C1 continuous 만족하는 알파에 대한 식입니다. 여기서 k는 윈도우의 크기이고, p는 트랜지션 모션의 프레임입니다. 두 번째로 식(2)는 루트 포지션을 블렌딩 하기 위한 식입니다. 앞서 말씀드린 것과 같이 앞 모션의 마지막 프레임과 뒤 모션에 추가된 프레임들에 대해 식(1)에서 구한 알파값으로 블렌딩을 진행합니다. 여기서 R_B_i는 뒤 모션에서 0부터 k-2까지의 프레임에 대하여 루트의 위치이고, R_A_last는 앞 모션에서 마지막 프레임 루트 관절의 좌표입니다. 마지막으로 식(3)은 관절들에 대해 회전값을 블렌딩하기 위한 식입니다. 여기서도 마찬가지로 앞 모션의 마지막 프레임 값만 사용하고, 알파에 대해 모든 관절들 끼리 slerp 합니다. 회전은 모든 관절들에 대해 블렌딩을 진행하므로 j는 관절의 번호이고, q_B_i는 뒤 모션에서 0부터 k-2까지의 프레임, q_A_last는 앞 모션에서 마지막 프레임입니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image15.png">
두 방법의 차이는 다음과 같이 확인 할 수 있습니다. 그림 (a)(b)에서 실린더로 표현된 궤적은 프레임 마다 비트가 되는 관절의 속도를 나타내는데 초록색에 가까울수록 큰 속도를 나타냅니다. 그림을 통해 알 수 있듯이 본 논문의 방법이 기존 방법보다 속도의 변화가 적고 부드럽게 이동하여 연결되는 비트를 더 강조 할 수 있습니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image16.png">
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/demo1.gif">
영상으로 확인하면 다음과 같습니다. 파란색을 보시면 앞 모션의 마지막 비트가 트랜지션 모션의 빠른 속도 때문에 잘 관측되지 않지만, 빨간색에서는 해당 동작의 비트를 강조하고 다음 모션의 비트로 넘어가는 것을 확인할 수 있습니다.
<br><br>
<img src="https://github.com/jong1-choi/MotionGraphsforDance/blob/main/Images/image17.png">
본 논문을 정리 하자면 춤 동작 데이터에서 비트를 탐색하여 마디를 추출합니다. 추출된 마디들로 모션 그래프를 생성합니다. 음악에 맞추어 마디의 구조 및 길이를 동기화 합니다. 마지막으로 연결되는 비트를 고려한 블렌딩 방법을 통해 변환 동작을 생성하여, 음악과 동기화 된 자연스러운 춤 동작을 생성 할 수 있습니다. 하지만 이러한 방법에도 한계가 있습니다. 먼저 속도의 크기에서 극솟값을 기준으로 비트를 추출하는 방법의 특성상 웨이브와 같이 지속적으로 움직이는 비트들은 검출이 어렵습니다. 또 춤 동작 데이터가 온전한 춤 동작이 아니라 준비 동작이 있는 경우나 하나의 데이터 안에 다른 주기를 갖는 여러 동작이 있는 경우 비트의 검출이 어렵습니다.
