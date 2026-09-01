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

---
English:
Helper functions for explicitly modeling Ego itself as a "moving car" within the CPD.

In the CPD of the paper (Scenario Modeling Language, Fig.2/Fig.7), Ego is treated
as a single car just like the other cars (LCar, RCar, etc.), and — following the
convention that "position is nothing but the ordering of the boxes" (concrete box,
paper Section IV-C, Pos(LCar(0))=0, etc.) — it has a simple straight-line chain of
boxes Ego(0) -> Ego(1) -> ... .

## Why simply adding a box chain for Ego is not enough

gcpd.py's add_trans imposes, at each step, an exclusive choice: either exactly one
of the registered ntrans/ctrans/... fires (i.e. exactly one car changes box), or
nothing happens (every car keeps the same value in every box as the previous step)
(`Or(ps_ntrans(...) + ... + [p_disable(...)])`). This is a straightforward
implementation of Definition 3's Petri-Net semantics, in which a single token
moves at a time; it can express "two cars advance independently, each on its own
separate step" (as with LCar/RCar in sample2.py), but it cannot express "two cars
advance simultaneously, on the same step" (p_disable only offers the single choice
"nobody moves" — "only A moves", "only B moves", and "A and B move together" can
only be selected if they are each registered as separate, distinct choices).

Consequently, if Ego were simply given its own independent box chain
(Ego(0)->Ego(1)->...) as an ntrans, Ego and the NPC would end up competing for the
same per-step "budget" as two independently-advancing cars (the same treatment as
LCar/RCar in sample2.py), and there would be no guarantee that Ego advances on
every single step (Ego would be held back on every step where the NPC's
transition happens to be the one chosen instead).

## The approach adopted: pacing Ego together with the NPC via synchronized transitions (strans)

We use the mechanism formally defined as "synchronized transition Es" in
Definition 3 of the paper (called `strans` in gcpd.py; this is the mechanism that
corresponds to the Fig.8 blind-spot example's description that "when the blue car
changes lane, the red car moves at the same time"). For every one of the NPC's
ntrans (transitions between distance zones / lanes), we mechanically generate a
synchronized-transition group meaning "whenever this NPC transition fires, Ego's
box chain also advances by one at the same time." The NPC's original ntrans are
replaced by their synchronized versions (so that they can no longer fire on their
own); this lets us keep the semantics consistent with CPD's event-driven meaning —
"whenever the NPC actually changes (zone or lane), Ego always advances by one at
that same moment" and "whenever the NPC does not change, Ego does not change
either (a step where nothing happens)" — while synchronizing Ego's advancement
with the NPC's behavior. (Because Ego's path is always a single straight line,
this does not change the set of satisfying solutions on the NPC side — the number
of scenarios stays the same as before Ego was added.)
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

    ---
    English:
    Returns a copy of model with a box chain for Ego added as a car, advancing
    one step at a time in sync with the NPC's transitions (the model argument
    itself is not modified).

    Args:
        model: an existing gcpd.Model that already has the NPC (etc.) as a car
            (the return value of logverify.reference_models.build_cutin_reference
            or logverify.multi_log_model.build_union_model). This function only
            synchronizes against model.ntrans (normal transitions) (because the
            current Method B/C models only use ntrans).
        max_step: the upper bound on the length of Ego's box chain
            (Ego(0)..Ego(max_step)). If None, model.max_step is used as-is
            (model.max_step must be finalized before calling this function —
            Method B's reference model stays at max_step=0 at construction
            time, so it needs to be finalized right before the membership
            check or visualization).
        ego_lane: the lane value assigned to Ego's boxes (a fixed value,
            default 0).
        ego_car: the car name assigned to Ego.
        npc_car: the NPC-side car name to synchronize against. If None,
            model.cars[0] is used.

    Returns:
        A new gcpd.Model with Ego added as a car.
    """
    ms = max_step if max_step is not None else model.max_step
    car = npc_car if npc_car is not None else model.cars[0]

    ego_boxes = list(range(ms + 1))  # Ego(0)..Ego(ms): 合計 ms 回まで前進できる
    # (English) Ego(0)..Ego(ms): can advance at most ms times in total.

    m = Model()
    m.set_car(list(model.cars) + [ego_car])
    m.set_box(list(model.boxes) + [(ego_car, i) for i in ego_boxes])
    m.set_position(list(model.position) + [(ego_car, i, i) for i in ego_boxes])
    m.set_lane(list(model.lane) + [(ego_car, i, ego_lane) for i in ego_boxes])
    m.set_init(list(model.inits) + [(ego_car, 0)])

    # NPC(等)の通常遷移は、Egoの前進と同期する版に置き換える
    # （単独では発火できないようにし、「NPCが動く瞬間 = Egoも動く瞬間」を強制する）。
    # (English) Replace the NPC's (etc.) normal transitions with versions
    # synchronized with Ego's advancement (so they cannot fire on their own,
    # forcing "the moment the NPC moves = the moment Ego also moves").
    strans: List[List[Tuple[str, int, str, int]]] = list(model.strans)
    for (c1, n1, c2, n2) in model.ntrans:
        for i in range(ms):
            strans.append([(c1, n1, c2, n2), (ego_car, i, ego_car, i + 1)])

    m.set_ntrans([])  # 元のntransは上のstransに吸収させる
    # (English) The original ntrans are absorbed into the strans above.
    m.set_ctrans(list(model.ctrans))
    m.set_netrans(list(model.netrans))
    m.set_cstrans(list(model.cstrans))
    m.set_strans(strans)

    m.max_step = ms
    m.num_model = model.num_model
    m.debug_const = model.debug_const
    m.debug_count = model.debug_count
    return m
