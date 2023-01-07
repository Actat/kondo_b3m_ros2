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

|   トピック名   |             型             | 内容                                                                                                               |
| :------------: | :------------------------: | :----------------------------------------------------------------------------------------------------------------- |
| ~/joint_states | sensor_msgs/msg/JointState | 各モータの情報です．effort は実測値ではなく目標値です．motor_list に記述されたモータの情報がパブリッシュされます． |

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
