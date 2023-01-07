[English](README.md) | [日本語](README.ja.md)

# kondo_b3m_ros2

近藤科学の[B3M シリーズ](https://kondo-robot.com/product-category/servomotor/b3m)のモータを ROS2 で扱うためのノードです．
パソコンとモータは[RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)を介して接続します．

# 使い方

```
mkdir -p ~/test_ws/src
cd ~/test_ws/src
git clone git@github.com:Actat/kondo_b3m_ros2.git
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

|    トピック名    |             型             | 内容                                                                                                                           |
| :--------------: | :------------------------: | :----------------------------------------------------------------------------------------------------------------------------- |
| /b3m_joint_state | sensor_msgs/msg/JointState | 各モータの position と velocity が含まれます．effort はありません．motor_list に記述されたモータの情報がパブリッシュされます． |

# service

|            サービス名             |                   型                    | 内容                                                                |
| :-------------------------------: | :-------------------------------------: | :------------------------------------------------------------------ |
|       /kondo_b3m_free_motor       |      kondo_b3m_ros2/srv/MotorFree       | モータがトルクを出さない状態にします．                              |
| /kondo_b3m_start_position_control | kondo_b3m_ros2/srv/StartPositionControl | モータが位置制御を開始します．動作ゲインはプリセット 0 になります． |
|  /kondo_b3m_start_speed_control   |  kondo_b3m_ros2/srv/StartSpeedControl   | モータが速度制御を開始します．動作ゲインはプリセット 1 になります． |
|    /kondo_b3m_desired_position    |   kondo_b3m_ros2/srv/DesiredPosition    | 位置制御の目標値を設定します．                                      |
|     /kondo_b3m_desired_speed      |     kondo_b3m_ros2/srv/DesiredSpeed     | 速度制御の目標値を設定します．                                      |

# parameter

パラメータはすべてオプションです．

| パラメータ名 | 型            | デフォルト値     | 内容                                                                                                                     |
| :----------: | :------------ | :--------------- | :----------------------------------------------------------------------------------------------------------------------- |
|  port_name   | String value  | `"/dev/ttyUSB0"` | RS485USB/シリアル変換アダプターのデバイスファイルの場所です．                                                            |
|   baudrate   | Integer value | `1500000`        | ボーレートです．モータの設定値に合わせてください．                                                                       |
|  motor_list  | String values | []               | joint state をパブリッシュするモータを指定するほか，回転方向やオフセットの設定が可能です．詳細は下記を参照してください． |

## motor_list の記述

motor_list は JSON string のリストです．
各モータごとに設定できる内容を表に示します．

| 設定項目  | 型      | デフォルト値 | 補足                                                                                                             |
| :-------: | :------ | :----------- | :--------------------------------------------------------------------------------------------------------------- |
|    id     | number  | --           | ID は必須です．                                                                                                  |
|   name    | string  | ID           | joint state に publish されるときの joint name です．指定しない場合，id が用いられます．                         |
|  offset   | number  | 0.0          | 原点のオフセットです．単位は rad です．                                                                          |
| direction | boolean | True         | False にすると回転方向を反転して処理します．モータの設定は変更しないため，ブロードキャスト指令は反転されません． |

以下に launch.py ファイルでの設定の例を示します．

```
return LaunchDescription([
    Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m',
        name='kondo_b3m',
        remappings=[('b3m_joint_state', 'joint_states')],
        parameters=[{'motor_list': [
            "{'id': 0}", # id must be set
            "{'id': 1, 'name': 'joint_1'}",
            "{'id': 2, 'offset': 0.2}",
            "{'id': 3, 'direction': False}",
            "{'id': 4, 'name': 'fifth_joint', 'offset': -0.5, 'direction': False}"
        ]}],
    )
])
```

# その他

モータの ID を変更するために`util_id_changer`を用意しています．
`ros2 run kondo_b3m_ros2 util_id_changer`で使用できます．
