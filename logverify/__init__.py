# logverify: ログ(Autoware/AJISAI)を CPD/GCPD (gcpd.py) の語彙に落とし込み、
# 「参照CPDモデル」に対する適合性検証(membership check)を行うためのモジュール群。
#
# 咲川氏の名前付き領域抽象化 (vendor/trajectory_abstraction/src/abstraction_*area.py) は
# 「ログを分類・要約する」ことに最適化された固定の抽象化スキームであり、
# 領域名 -> (lane, position) の変換に任意のルックアップ表 (cpd_bridge.REGION_TO_LANE_POS)
# を挟む必要がある。
#
# 一方、cut-in を表す「参照CPD」を自分たちで直接書き下し、ログがそれを充足するかどうかを
# SAT/UNSATで判定したい場合は、参照CPDを定義するときに使った (lane, position) の粒度と
# ログの離散化の粒度を最初から一致させておく方が自然かつ厳密である。
#
# grid_bridge.py はまさにその目的のための「格子(グリッド)ベースの抽象化」を提供する:
# 縦方向・横方向のセルサイズ (gx, gy) を選ぶと、ログのどの時刻についても
# 整数の (position=i, lane=k) がそのまま得られる。参照CPDモデル (reference_models.py) は
# 同じ (gx, gy) を前提にした box グラフとして書かれており、
# membership.check_membership() で両者を突き合わせる。
#
# (English)
# logverify: a collection of modules that translate logs (Autoware/AJISAI) into
# the vocabulary of CPD/GCPD (gcpd.py), and perform conformance checking
# (membership check) against a "reference CPD model".
#
# Sakikawa's named-region abstraction (vendor/trajectory_abstraction/src/abstraction_*area.py)
# is a fixed abstraction scheme optimized for "classifying and summarizing logs",
# and requires an arbitrary lookup table (cpd_bridge.REGION_TO_LANE_POS) to be
# inserted for converting region names -> (lane, position).
#
# On the other hand, when we want to write down a "reference CPD" representing a
# cut-in directly ourselves and judge SAT/UNSAT as to whether a log satisfies it,
# it is more natural and rigorous to align the granularity of the log's
# discretization with the granularity of (lane, position) used when defining the
# reference CPD, from the very start.
#
# grid_bridge.py provides exactly a "grid-based abstraction" for that purpose:
# once the vertical/horizontal cell sizes (gx, gy) are chosen, an integer
# (position=i, lane=k) is obtained directly for any time in the log. The
# reference CPD model (reference_models.py) is written as a box graph that
# assumes the same (gx, gy), and membership.check_membership() reconciles the
# two.
