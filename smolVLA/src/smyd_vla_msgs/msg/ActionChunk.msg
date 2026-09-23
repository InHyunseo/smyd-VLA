# 한 번의 추론으로 나온 action 여러 스텝. header.stamp는 예측의 기준이 된 관측 시각이다.
std_msgs/Header header
float32 seconds_per_step
uint32 action_dimension
float32[] actions          # 스텝 순서로 이어 붙인 값 (길이 = 스텝 수 * action_dimension)
