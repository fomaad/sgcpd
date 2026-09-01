"""
Ego自身をCPDの中で「動く車」として明示的にモデル化するための補助関数。

論文（Scenario Modeling Language, Fig.2/Fig.7）のCPDでは、Egoも他の車
（LCar, RCarなど）と同じく1つの car として扱われ、"position は箱の並び順
そのもの" という規約（concrete box、論文IV-C節、Pos(LCar(0))=0 等）に
基づく単純な直線の箱列 Ego(0) -> Ego(1) -> ... を持つ。

## なぜ単純にEgoの箱列を追加するだけでは足りないか

gcpd.py の add_trans は、1ステップにつき「登録されているntrans/ctrans/...
のうちどれか1つだけが発火する（＝1台の車だけが箱を変える）か、
何も起きない（全car・全箱が前ステップと同じ値を保つ）か」という
排他的選択を各ステップに課している（`Or(ps_ntrans(...) + ... +
[p_disable(...)])`。これはPetri Netのトークンが1つずつ移動する
という定義3の素朴な実装であり、sample2.pyのLCar/RCarのように
「2台が独立に、互いに異なるステップで進む」ことは表現できるが、
「2台が同じステップで同時に進む」ことは表現できない
（p_disableは「誰も動かない」の1択しかなく、「Aだけ動く」
「Bだけ動く」と「AとBが同時に動く」は別々の選択肢として
登録しない限り選べない）。

そのため、単純にEgoにも独立した箱列（Ego(0)->Ego(1)->...）を
ntransとして追加するだけだと、EgoとNPCは同じステップ予算を
奪い合う「独立に進む2台の車」になってしまい
（sample2.pyのLCar/RCarと同じ扱い）、Egoが毎ステップ確実に
前進する保証がなくなる（NPCの遷移が選ばれた回のぶんだけ
Egoは足止めされる）。

## 採用した方法: 同期遷移 (strans) によるEgoとNPCの歩調合わせ

論文の定義3で「同期遷移 Es」として正式に定義されている機構
（gcpd.pyでは `strans`。Fig.8のblind spot事例で「青い車が車線変更すると
同時に赤い車も動く」という記述に対応する仕組み）を使い、
NPC側の全てのntrans（各距離帯・車線間の遷移）それぞれに対して、
「そのNPCの遷移が発火するときは、Egoの箱列も同時に1つ進む」という
同期遷移グループを機械的に生成する。NPC側の元のntransは同期版に
置き換える（単独では発火できないようにする）ため、
「NPCが実際に(zoneやlaneを)変える瞬間には必ずEgoも1つ前進する」
「NPCが変化しない瞬間はEgoも変化しない（何も起きないステップ）」
という、CPDのイベント駆動な意味論と矛盾しない形で、
Egoの前進をNPCの挙動と同期させることができる
（Egoの経路は常に一本道なので、これによってNPC側の充足解の
集合が変わることはない。scenario数はEgoを加える前と同じになる）。
"""

from typing import List, Optional, Tuple

from gcpd import Model


def with_ego_track(
    model: Model,
    max_step: Optional[int] = None,
    ego_lane: int = 0,
    ego_car: str = "Ego",
    npc_car: Optional[str] = None,
) -> Model:
    """model のコピーに、NPCの遷移と同期して1つずつ前進するEgoの箱列を
    car として追加して返す（引数の model 自体は変更しない）。

    Args:
        model: NPC（等）を car として持つ既存の gcpd.Model
            （logverify.reference_models.build_cutin_reference や
            logverify.multi_log_model.build_union_model の戻り値）。
            本関数は model.ntrans（通常遷移）のみを同期対象とする
            （現状のMethod B/Cのモデルはntransしか使っていないため）。
        max_step: Egoの箱列の長さの上限（Ego(0)..Ego(max_step)）。
            None の場合は model.max_step をそのまま使う
            （呼び出し前に model.max_step を確定させておくこと。
            方法Bの参照モデルは構築時点では max_step=0 のままなので、
            membership check や可視化の直前に確定させる必要がある）。
        ego_lane: Egoの箱に割り当てるlaneの値（固定値。デフォルト0）。
        ego_car: Egoに割り当てるcar名。
        npc_car: 同期対象のNPC側car名。Noneの場合は model.cars[0] を使う。

    Returns:
        Ego を car として追加した新しい gcpd.Model。
    """
    ms = max_step if max_step is not None else model.max_step
    car = npc_car if npc_car is not None else model.cars[0]

    ego_boxes = list(range(ms + 1))  # Ego(0)..Ego(ms): 合計 ms 回まで前進できる

    m = Model()
    m.set_car(list(model.cars) + [ego_car])
    m.set_box(list(model.boxes) + [(ego_car, i) for i in ego_boxes])
    m.set_position(list(model.position) + [(ego_car, i, i) for i in ego_boxes])
    m.set_lane(list(model.lane) + [(ego_car, i, ego_lane) for i in ego_boxes])
    m.set_init(list(model.inits) + [(ego_car, 0)])

    # NPC(等)の通常遷移は、Egoの前進と同期する版に置き換える
    # （単独では発火できないようにし、「NPCが動く瞬間 = Egoも動く瞬間」を強制する）。
    strans: List[List[Tuple[str, int, str, int]]] = list(model.strans)
    for (c1, n1, c2, n2) in model.ntrans:
        for i in range(ms):
            strans.append([(c1, n1, c2, n2), (ego_car, i, ego_car, i + 1)])

    m.set_ntrans([])  # 元のntransは上のstransに吸収させる
    m.set_ctrans(list(model.ctrans))
    m.set_netrans(list(model.netrans))
    m.set_cstrans(list(model.cstrans))
    m.set_strans(strans)

    m.max_step = ms
    m.num_model = model.num_model
    m.debug_const = model.debug_const
    m.debug_count = model.debug_count
    return m
