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

[RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)を`/dev/ttyKONDO`として認識させます．

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

|   トピック名   |             型             | 内容                                                                                                                         |
| :------------: | :------------------------: | :--------------------------------------------------------------------------------------------------------------------------- |
| ~/joint_states | sensor_msgs/msg/JointState | 各モータの情報です．effort は負荷電流とトルク定数に基づく値です．motor_list に記述されたモータの情報がパブリッシュされます． |

# service

|   サービス名   |               型               | 内容                                       |
| :------------: | :----------------------------: | :----------------------------------------- |
| ~/control_mode | kondo_b3m_ros2/srv/ControlMode | モータの制御を開始したり終了したりします． |
|   ~/desired    |   kondo_b3m_ros2/srv/Desired   | モータに制御の目標値を送ります．           |

# parameter

パラメータはすべてオプションです．

|   パラメータ名    | 型            | デフォルト値      | 内容                                                                                                                                           |
| :---------------: | :------------ | :---------------- | :--------------------------------------------------------------------------------------------------------------------------------------------- |
|     port_name     | String value  | `"/dev/ttyKONDO"` | RS485USB/シリアル変換アダプターのデバイスファイルの場所です．                                                                                  |
|     baudrate      | Integer value | `1500000`         | ボーレートです．モータの設定値に合わせてください．                                                                                             |
| publish_frequency | int           | `50`              | `~/joint_states`にパブリッシュする周波数（Hz）です．                                                                                           |
|    motor_list     | String values | []                | ここに記述したモータの情報が`~/joint_states`にパブリッシュされます．また，回転方向やオフセットの設定も可能です．詳細は下記を参照してください． |

## motor_list の記述

motor_list は JSON string のリストです．
各モータごとに設定できる内容を表に示します．

| 設定項目  | 型      | デフォルト値 | 補足                                                                                                             |
| :-------: | :------ | :----------- | :--------------------------------------------------------------------------------------------------------------- |
|    id     | number  | --           | ID は必須です．                                                                                                  |
|   model   | string  | --           | joint state の effort の計算に用いるトルク定数を設定するために必要です．                                         |
|   name    | string  | ID           | joint state に publish されるときの joint name です．指定しない場合，id が用いられます．                         |
|  offset   | number  | 0.0          | 原点のオフセットです．単位は rad です．                                                                          |
| direction | boolean | True         | False にすると回転方向を反転して処理します．モータの設定は変更しないため，ブロードキャスト指令は反転されません． |

以下に launch.py ファイルでの設定の例を示します．

```
kondo_b3m_ros2_node = Node(
    package='kondo_b3m_ros2',
    executable='kondo_b3m',
    remappings=[('b3m_joint_state', 'joint_states')],
    parameters=[{
        'port_name': '/dev/ttyKONDO',
        'baudrate': 1500000,
        'publish_frequency': 50,
        'motor_list': ['{"id": 0, "model": "B3M-SC-1170-A", "name": "joint0", "direction": true, "offset": 0}',
                       '{"id": 1, "model": "B3M-SB-1040-A"}',
                       '{"id": 2, "name": "joint2"}',
                       '{"id": 3, "direction": false}',
                       '{"id": 4, "offset": 0.5}',
                       '{"id": 5}'],
    }]
)
```
