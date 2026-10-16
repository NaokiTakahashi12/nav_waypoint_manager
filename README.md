# ROS 2 Waypoint Manager

RViz で位置目標を登録・編集し、Waypoint と訪問順序を保存するパッケージ群です。
経路の進行は TF 上のロボット位置と到達半径で判断します。Nav2 固有の Action、
成功・失敗・キャンセル状態には依存しません。出力目標を受け取った後の移動制御は、
接続先のナビゲーションシステムが担当します。

## 起動と保存

ROS 2 Humble でビルドできます。

```bash
colcon build --packages-up-to nav_waypoint_manager
source install/setup.bash
ros2 launch nav_waypoint_bringup waypoint_manager.launch.py use_rviz:=true
```

RViz の SetGoal ツールは `/set_waypoint` に配信し、Waypoint を登録します。
Marker を移動・回転してマウスを離すと位置を確定します。
目標の登録だけでは訪問順序に追加されません。経路は別途設定してください。

```bash
ros2 run nav_waypoint_bringup waypoint_edit upsert entrance --position 1 0 0 --yaw 0
ros2 run nav_waypoint_bringup waypoint_edit upsert desk --position 4 2 0 \
  --property goal_reached_radius=0.25 --property stop_with_resume=true
ros2 run nav_waypoint_bringup waypoint_edit set-route entrance desk
ros2 run nav_waypoint_bringup waypoint_edit save
ros2 topic pub --once /nav_route_server/release_resume std_msgs/msg/Bool '{data: true}'
```

launch の入力ファイルは従来の `waypoints_file` と `route_file` です。既定では
パッケージのサンプルを読み込みます。保存先は別の `waypoints_save_file` と
`route_save_file` で指定でき、launch の既定値は
`${XDG_DATA_HOME:-$HOME/.local/share}/nav_waypoint_manager/` 配下です。
保存した内容を次回起動時に使う場合は、入力ファイルもその保存先に指定してください。
ノードを直接起動した場合、保存先パラメーターが空なら入力ファイルを保存先に使います。
指定した保存先の親ディレクトリは事前に作成してください。launch は既定の保存用
ディレクトリだけを作成します。

保存は同じディレクトリの一時ファイルへ書き込み、flush 後に置き換えます。
サービス応答の `success` と `message` で結果を確認できます。
Waypoint と Route の保存は別々の操作です。`waypoint_edit save` でも、
片方だけ成功する可能性があるため、エラーが出た場合は内容を確認して再実行してください。
同じ YAML を保存先に使う場合は、他のセクションを保ったまま該当セクションを更新します。
別プロセスから同一保存ファイルを同時に書き込む運用は避けてください。

## 経路進行の仕様

- 起動時、reset 後、経路の置き換え後は、開始指示を待ちます。
  `route_index_auto_reset=false` の場合は、reset も必要です。
- Waypoint 一覧の再配信や位置・属性の編集では、経路番号をリセットしません。
  更新は次の観測処理から反映します。訪問済みの点は再訪問しません。
- 到達条件は距離が半径以下であることです。姿勢は到達条件に含みません。
  `evalute_only_xy_reach=true` では Z を無視します。
- 通過用の点は、開始時から半径内なら目標を配信せず、次へ進みます。
- `stop_with_resume=true` の点は、半径内に入ると再開待ちになります。
  開始時から半径内でも、その点の目標を配信し、開始指示とは別の再開指示を待ちます。
- 再開待ち中も、同じ位置目標を周期配信します。ロボットが半径外へ移動しても
  待機は維持し、再開指示と到達条件の両方が成立したときに次へ進みます。
  この機能はロボットへの停止・キャンセル命令ではありません。
- 開始待ち・再開待ちで受け付けた `release_resume=true` は、TF が取得できない間も
  保持します。移動中の指示は無視し、将来の再開待ちには持ち越しません。
  `false` は操作しません。
- TF を取得できない間は進行を保留します。開始済みなら現在の目標の再配信を続けます。
- 存在しない Waypoint を自動で飛ばしません。経路全体の参照を検証し、
  不整合があれば進行を保留します。有効な一覧を受信すると再検証します。
- 最終点の条件を満たすと `completed` となり、目標配信を終了します。
  これは経路上の位置条件を満たしたことを示し、ナビゲーション成功を意味しません。

受信側は周期配信される同じ目標を扱える必要があります。また、配信が終了しても
最後の目標を取り消す命令は送信しません。起動・reset・経路編集・完了時に、
ロボットの物理的な停止が必要な場合は外部の操作系で制御してください。

`reach_observe_frequency` は有限の正数で最大 1000 Hz、到達半径は有限の正数です。
`publish_goal_with_reach_observe=N` は `max(1, N) / reach_observe_frequency` 秒以上の
間隔で配信します。0 と 1 は毎回の観測を上限とします。一覧の再配信や無関係な点の
編集では間隔をリセットしません。現在の目標の変更時は次の観測で配信します。
経路検証・到達属性の解析はデータ更新時に行い、周期観測では名前索引を使います。設定パラメーターは起動時に確定し、実行中は変更できません。

`/nav_route_server/progress` は最新の経路進行を transient local で公開します。
状態は `disabled`、`empty`、`startup_wait`、`running`、`waiting_resume`、
`blocked`、`completed` です。経路番号は 0 始まりで、完了時は経路サイズに等しくなります。
`reason` は TF や参照不整合など、進行を保留している理由です。

## 編集 API

| インターフェース | 型 | 用途 |
| --- | --- | --- |
| `/nav_waypoint_server/upsert` | `nav_waypoint_msgs/srv/UpsertWaypoint` | 位置・属性を含む Waypoint の登録・置換 |
| `/nav_waypoint_server/delete` | `nav_waypoint_msgs/srv/DeleteWaypoint` | 経路から参照されていない点の削除 |
| `/nav_waypoint_server/save` | `std_srvs/srv/Trigger` | Waypoint 保存と結果通知 |
| `/nav_waypoint_server/publish` | `std_srvs/srv/Empty` | 一覧の再配信 |
| `/nav_route_server/set_route` | `nav_waypoint_msgs/srv/SetRoute` | 訪問順序の置換。空配列で消去 |
| `/nav_route_server/save` | `std_srvs/srv/Trigger` | 経路保存と結果通知 |
| `/nav_route_server/reset` | `std_srvs/srv/Empty` | 先頭へ戻し、開始待ちにする |
| `/nav_route_server/release_resume` | `std_msgs/msg/Bool` | 開始・再開指示 |

従来の `/nav_waypoint_server/regist` と `/modify` トピックも利用できます。
検証に失敗したトピック入力はログを出して拒否します。結果が必要な編集にはサービスを
使ってください。属性は全体を置き換えます。未知の属性も保持しますが、キーの重複を
拒否し、既知の `goal_reached_radius` と `stop_with_resume` は型と値を検証します。

```bash
ros2 run nav_waypoint_bringup waypoint_edit upsert desk --property goal_reached_radius=0.5
ros2 run nav_waypoint_bringup waypoint_edit upsert desk --remove-property stop_with_resume
ros2 run nav_waypoint_bringup waypoint_edit rename desk reception
ros2 run nav_waypoint_bringup waypoint_edit set-route entrance
ros2 run nav_waypoint_bringup waypoint_edit delete reception
```

名前変更は新しい点の登録、経路参照の置換、旧点の削除の順に行います。
経路内で同じ点が複数回使われている場合も置換します。経路を変更すると開始待ちになります。
参照のない点の名前変更、および同じ訪問順序の `set_route` は進行状態を保持します。
待機中の目標位置・到達半径・停止属性を変更すると、変更前の待機と再開要求を解除します。
複数ノード間の変更はトランザクションではありません。途中で失敗した場合は
新しい点が残ることがあるため、一覧と経路を確認して操作を完了してください。
同時に複数の編集クライアントから名前変更・経路変更を実行しないでください。
名前空間を使用する場合は `waypoint_edit --namespace /robot1 ...` と指定します。

経路参照は内部サービス `/nav_waypoint_server/reserve_route` で Waypoint server に
確保します。経路更新の成功応答後は、トピック配送前でも参照点を削除できません。
複数の Route server を使う場合は異なるノード名にしてください。削除保護は全 server の
参照の和集合です。終了する server の参照は終了前に空の `set_route` で解放してください。
起動時に参照確保ができない場合は `blocked` とし、削除も参照情報が届くまで拒否します。
参照情報はメモリー上にあるため、Waypoint server を再起動するときは Route server も
一緒に再起動してください。標準 launch は server 終了時に全体を終了します。

保存は受付時のスナップショットを専用ワーカーで書き込み、完了後に応答します。
ディスク待ちで経路観測や再開受付を止めません。待ち行列は各 server 最大 4 件で、
満杯の場合は失敗応答を返します。保存中の編集は次回の保存対象になります。

## 配信・到達判定のオプション

`max_transform_age` は動的 TF の最大経過時間で、既定値は 1 秒です。
`future_transform_tolerance` は未来のタイムスタンプの許容値で、既定値は 0.5 秒です。
範囲外では到達判定を保留し、再開要求を保持します。時刻 0 の静的 TF は期限判定を
免除します。シミュレーションでは各ノードの `use_sim_time` を揃えてください。

半径を省略した点には Waypoint server の `default_goal_reached_radius`（既定値 1 m）を
明示的に付与して配信・保存します。launch の `reach_position_threshold` はこの既定値と
Route server の従来のフォールバック値を揃えます。変換ノードは半径を固定しません。

同じ位置命令を受けるたびに再計画する受信側では、launch 引数
`publish_unchanged_goals:=false` を指定できます。出力変換ノードが名前・姿勢・フレームの
同じ命令を抑制します。既定値は `true` で従来の周期配信を維持します。
抑制を有効にした場合、受信側の再起動や同じ目標への明示的な再送も自動検出しません。
Nav2 の状態・成功・失敗・キャンセルの取得は追加していません。

表示は名前索引で差分を判定し、同一内容の更新を無視します。経路 Marker は不要になった
ID だけを削除します。後から接続した RViz にも全体を表示するため、内容変更時の
transient local 配信には現在の全 Marker を含めます。

## 座標系と互換性

`waypoint_frame_id` は両 server と変換ノードで共通です。異なる座標系の入力は拒否し、
自動変換はしません。変換ノードは従来の `map` 固定ではなく共通設定を使います。
位置の非有限値と非単位 Quaternion を拒否します。

`Waypoint.msg` に `frame_id` を追加しました。空なら server の設定フレームとして
扱いますが、配信と保存ではフレームを明示します。既存の YAML に `frame_id` がない場合は
起動パラメーターのフレームを使います。新しい保存データは
`waypoint_manager.frame_id` を持ち、起動設定との不一致を拒否します。
`properties: null` と属性省略も読み込めます。

**インターフェース変更:** `Waypoint.msg` の変更により、依存パッケージも再ビルドが
必要です。Waypoint 保存サービスの型は `Empty` から `Trigger` に変更しました。
呼び出し側も更新してください。

```bash
ros2 service call /nav_waypoint_server/save std_srvs/srv/Trigger '{}'
ros2 service call /nav_route_server/save std_srvs/srv/Trigger '{}'
```

## 検証

```bash
colcon build --packages-up-to nav_waypoint_manager --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select nav_waypoint_bringup
colcon test-result --verbose
```

回帰テストは独立した ROS domain と localhost 通信で各ノードを起動し、TF を入力します。
実機や Nav2 を必要とせず、編集後の進行位置、再開待ち、保存結果、座標系、
Marker の編集確定、経路参照、CLI による名前変更を確認します。
