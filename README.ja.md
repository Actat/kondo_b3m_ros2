[English](README.md) | [日本語](README.ja.md)

# kondo_b3m_ros2

近藤科学の[B3M シリーズ](https://kondo-robot.com/product-category/servomotor/b3m)のモータを ROS2 で扱うためのノードです．
パソコンとモータは[RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)を介して接続します．

# 使い方

このリポジトリに加えて[kondo_b3m_interfaces](https://github.com/Actat/kondo_b3m_interfaces)が必要です．

```
mkdir -p ~/test_ws/src
cd ~/test_ws/src
git clone git@github.com:Actat/kondo_b3m_ros2.git
git clone git@github.com:Actat/kondo_b3m_interfaces.git
```

[RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)を`/dev/ttyUSB0`として認識させます．

```
sudo cp ~/test_ws/src/kondo_b3m_ros2/config/99-kondo-rs485.rules /etc/udev/rules.d/
```

コンパイルして実行します．

```
cd ~/test_ws
colcon build
source install/setup.bash
ros2 run kondo_b3m_ros2 kondo_b3m
```

# パブリッシュされる topic

|    トピック名    |             型             | 内容                                                                                                                         |
| :--------------: | :------------------------: | :--------------------------------------------------------------------------------------------------------------------------- |
| /b3m_joint_state | sensor_msgs/msg/JointState | 各モータの position と velocity が含まれます．effort はありません．ジョイント名は`joint_name_list`パラメータで指定できます． |

# service

|            サービス名             |                      型                       | 内容                                                                |
| :-------------------------------: | :-------------------------------------------: | :------------------------------------------------------------------ |
|       /kondo_b3m_free_motor       |      kondo_b3m_interfaces/srv/MotorFree       | モータがトルクを出さない状態にします．                              |
| /kondo_b3m_start_position_control | kondo_b3m_interfaces/srv/StartPositionControl | モータが位置制御を開始します．動作ゲインはプリセット 0 になります． |
|  /kondo_b3m_start_speed_control   |  kondo_b3m_interfaces/srv/StartSpeedControl   | モータが速度制御を開始します．動作ゲインはプリセット 1 になります． |
|    /kondo_b3m_desired_position    |   kondo_b3m_interfaces/srv/DesiredPosition    | 位置制御の目標値を設定します．                                      |
|     /kondo_b3m_desired_speed      |     kondo_b3m_interfaces/srv/DesiredSpeed     | 速度制御の目標値を設定します．                                      |

# parameter

パラメータはすべてオプションです．

|     パラメータ名     | 型             | デフォルト値                 | 内容                                                                                                        |
| :------------------: | :------------- | :--------------------------- | :---------------------------------------------------------------------------------------------------------- |
|      port_name       | String value   | `"/dev/ttyUSB0"`             | RS485USB/シリアル変換アダプターのデバイスファイルの場所です．                                               |
|       baudrate       | Integer value  | `1500000`                    | ボーレートです．モータの設定値に合わせてください．                                                          |
|   joint_name_list    | String values  | `std::vector<std::string>{}` | このリストの ID 番目の要素を`/b3m_joint_states`へ出力する`sensor_msgs/JointState Message`の`name`にします． |
| joint_direction_list | Boolean values | `std::vector<bool>{}`        | 軸の回転方向を切り替えられます．`true`の場合と設定しない場合は正回転，`false`の場合は逆回転になります．     |
|  joint_offset_list   | Double values  | `std::vector<double>{}`      | 軸の角度にオフセットを設定できます．単位は rad です．                                                       |

# 使用上の注意

接続するモータの ID を 0, 1, 2, ..., n のようなゼロ始まりの連番にしてください．
`joint_name_list`や`joint_direction_list`に関係する処理で不具合を起こす可能性があります．
モータの ID を変更するために`util_id_changer`を用意しています．
`ros2 run kondo_b3m_ros2 util_id_changer`で使用できます．
