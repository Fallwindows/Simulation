"""Generate the fictional retail asset library used by the Isaac runtime.

The runtime library is intentionally procedural and self-contained, but the
generated assets are authored as small product assemblies rather than colored
placeholder primitives: packages have chamfered shells and label panels,
bottles have shoulders/necks/caps, cans and jars have rims/lids, and produce is
shaped and shaded as a natural display object.  The same catalog is consumed by
the deterministic layout builder, so visual refinements never change semantic
placement decisions.
"""

from __future__ import annotations

import json
import hashlib
import math
import struct
import zlib
from dataclasses import dataclass, asdict
from pathlib import Path

try:
    import PIL
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover - fail-closed validation reports this runtime mismatch
    PIL = Image = ImageDraw = ImageFont = None


REQUIRED_PILLOW_VERSION = "12.3.0"
REQUIRED_ZLIB_VERSION = "1.3.2"
REQUIRED_FONTS = {
    False: (
        Path(r"C:\Windows\Fonts\segoeui.ttf"),
        "8134dbcd09e7b123c9a7f229d49cffbcb01352cc72ea5e1076b65d0dca9f73cd",
    ),
    True: (
        Path(r"C:\Windows\Fonts\segoeuib.ttf"),
        "aeb9e4a6ec5cc59f4d72df8189032d7dbb28f45161cf1552174818b5465dac4e",
    ),
}


def _validate_texture_toolchain() -> None:
    """Fail before writing when the byte-reproducible texture stack is absent."""
    errors: list[str] = []
    if PIL is None or Image is None or ImageDraw is None or ImageFont is None:
        errors.append(f"Pillow=={REQUIRED_PILLOW_VERSION} is required")
    elif PIL.__version__ != REQUIRED_PILLOW_VERSION:
        errors.append(f"Pillow=={REQUIRED_PILLOW_VERSION} required; found {PIL.__version__}")
    if zlib.ZLIB_RUNTIME_VERSION != REQUIRED_ZLIB_VERSION:
        errors.append(f"zlib runtime {REQUIRED_ZLIB_VERSION} required; found {zlib.ZLIB_RUNTIME_VERSION}")
    for _, (path, expected_hash) in REQUIRED_FONTS.items():
        if not path.is_file():
            errors.append(f"required font missing: {path}")
            continue
        actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual_hash != expected_hash:
            errors.append(f"required font hash mismatch: {path} ({actual_hash})")
    if errors:
        raise RuntimeError(
            "Retail texture generation prerequisites are not reproducible:\n- "
            + "\n- ".join(errors)
            + "\nSee tools/retail_assets/GENERATION_REQUIREMENTS.md. No assets were generated."
        )


@dataclass(frozen=True)
class AssetSpec:
    asset_key: str
    category: str
    dimensions_m: tuple[float, float, float]
    model_type: str
    color: tuple[float, float, float]
    accent: tuple[float, float, float]
    product_name: str
    department: str = "pantry"
    intended_support: str = "shelf"
    assembly_parts: tuple[str, ...] = ()


def _spec(
    key: str,
    category: str,
    dims: tuple[float, float, float],
    model: str,
    color: tuple[int, int, int],
    accent: tuple[int, int, int],
    name: str,
    department: str = "pantry",
    intended_support: str = "shelf",
    parts: tuple[str, ...] = (),
) -> AssetSpec:
    return AssetSpec(
        key, category, dims, model,
        tuple(v / 255.0 for v in color),
        tuple(v / 255.0 for v in accent),
        name, department, intended_support, parts,
    )


ASSET_SPECS: tuple[AssetSpec, ...] = (
    _spec("cereal_sunrise", "cereal", (0.13, 0.10, 0.31), "box", (242, 168, 46), (39, 94, 155), "SUNRISE OATS"),
    _spec("cereal_harvest", "cereal", (0.13, 0.10, 0.31), "box", (205, 76, 53), (244, 208, 75), "HARVEST LOOP"),
    _spec("cereal_grain", "cereal", (0.13, 0.10, 0.30), "box", (116, 164, 77), (236, 224, 159), "GRAIN DAY"),
    _spec("cereal_berry", "cereal", (0.13, 0.10, 0.31), "box", (92, 87, 160), (245, 117, 131), "BERRY CLOUD"),
    _spec("cereal_honey", "cereal", (0.13, 0.10, 0.31), "box", (228, 153, 38), (108, 68, 31), "HONEY CRUNCH"),
    _spec("cereal_morning", "cereal", (0.13, 0.10, 0.30), "box", (57, 142, 158), (239, 225, 169), "MORNING MIX"),
    _spec("snack_cracker", "snacks", (0.13, 0.09, 0.24), "box", (224, 133, 48), (121, 49, 35), "CRISP STACK"),
    _spec("snack_pretzel", "snacks", (0.13, 0.09, 0.24), "box", (180, 83, 43), (245, 201, 80), "PRETZEL POP"),
    _spec("snack_chips", "snacks", (0.13, 0.09, 0.25), "box", (65, 132, 87), (246, 208, 74), "GARDEN CRUNCH"),
    _spec("snack_wafer", "snacks", (0.13, 0.09, 0.23), "box", (128, 76, 145), (246, 189, 219), "WAFER WAVE"),
    _spec("snack_popcorn", "snacks", (0.13, 0.09, 0.24), "box", (237, 209, 74), (49, 91, 158), "POPCORN FIELD"),
    _spec("juice_citrus", "juice", (0.13, 0.11, 0.28), "carton", (230, 122, 38), (246, 222, 112), "CITRUS PRESS"),
    _spec("juice_berry", "juice", (0.13, 0.11, 0.28), "carton", (114, 68, 142), (236, 158, 195), "BERRY POUR"),
    _spec("juice_green", "juice", (0.13, 0.11, 0.28), "carton", (64, 139, 94), (207, 232, 133), "GREEN GROVE"),
    _spec("juice_apple", "juice", (0.13, 0.11, 0.28), "carton", (193, 69, 54), (247, 219, 103), "APPLE PRESS"),
    _spec("water_sky", "water", (0.10, 0.10, 0.28), "bottle", (72, 147, 206), (210, 239, 252), "SKY WATER"),
    _spec("water_clear", "water", (0.10, 0.10, 0.28), "bottle", (71, 174, 157), (223, 247, 235), "CLEAR SPRING"),
    _spec("soda_orbit", "soda", (0.10, 0.10, 0.27), "bottle", (54, 91, 177), (238, 220, 85), "ORBIT FIZZ"),
    _spec("soda_cherry", "soda", (0.10, 0.10, 0.27), "bottle", (186, 47, 57), (246, 185, 83), "CHERRY SPARK"),
    _spec("soda_lime", "soda", (0.10, 0.10, 0.27), "bottle", (93, 155, 62), (225, 240, 131), "LIME LIFT"),
    _spec("can_soup_red", "cans", (0.11, 0.11, 0.13), "can", (187, 55, 45), (246, 205, 91), "HEARTH SOUP"),
    _spec("can_soup_green", "cans", (0.11, 0.11, 0.13), "can", (67, 125, 77), (232, 218, 132), "HERB SOUP"),
    _spec("can_beans", "cans", (0.11, 0.11, 0.13), "can", (69, 94, 159), (239, 187, 81), "BLUE BEANS"),
    _spec("can_tomato", "cans", (0.11, 0.11, 0.13), "can", (208, 77, 46), (239, 236, 184), "TOMATO TABLE"),
    _spec("jar_honey", "jars", (0.12, 0.12, 0.18), "jar", (218, 157, 47), (244, 222, 119), "GOLDEN HIVE"),
    _spec("jar_pickle", "jars", (0.12, 0.12, 0.18), "jar", (74, 139, 86), (210, 235, 151), "DILL GARDEN"),
    _spec("jar_sauce", "jars", (0.12, 0.12, 0.18), "jar", (177, 64, 47), (244, 198, 90), "SAUCE HOUSE"),
    _spec("produce_crate_green", "produce_crate", (0.44, 0.32, 0.22), "crate", (110, 137, 68), (223, 175, 66), "FARM CRATE"),
    _spec("produce_crate_red", "produce_crate", (0.44, 0.32, 0.22), "crate", (159, 77, 55), (236, 194, 73), "MARKET CRATE"),
    # Grocery produce needs to read as produce at first-person distance, not
    # as sub-pixel decoration inside the bins.
    _spec("red_apple", "red_apples", (0.20, 0.20, 0.20), "fruit", (190, 46, 47), (242, 168, 70), "RED APPLE"),
    _spec("green_apple", "green_apples", (0.20, 0.20, 0.20), "fruit", (92, 159, 67), (236, 202, 75), "GREEN APPLE"),
    _spec("orange", "oranges", (0.21, 0.21, 0.21), "fruit", (230, 128, 35), (247, 213, 90), "ORANGE"),
    _spec("lemon", "lemons", (0.20, 0.20, 0.18), "fruit", (238, 205, 58), (110, 154, 67), "LEMON"),
    _spec("promo_market_sign", "promotional_sign", (0.92, 0.05, 0.48), "sign", (49, 105, 153), (243, 187, 62), "FRESH MARKET"),
    # New physical assemblies.  Each profile has different authored parts,
    # proportions, and silhouette; these are inventory types rather than
    # label/color variants of the legacy package families above.
    _spec("milk_gallon", "milk", (0.18, 0.13, 0.29), "milk_jug", (238, 241, 232), (49, 113, 184), "MEADOW MILK", "refrigerated", parts=("jug_body", "shoulder", "offset_neck", "handle", "cap")),
    _spec("milk_carton", "milk", (0.10, 0.09, 0.24), "gable_carton", (236, 242, 235), (52, 142, 105), "VALLEY MILK", "refrigerated", parts=("carton_body", "gable_left", "gable_right", "top_seam", "pour_cap")),
    _spec("yogurt_cup", "refrigerated", (0.12, 0.12, 0.11), "yogurt_cup", (239, 231, 224), (150, 66, 118), "BERRY YOGURT", "refrigerated", parts=("tapered_cup", "foil_lid", "base_ring")),
    _spec("egg_carton", "refrigerated", (0.29, 0.11, 0.075), "egg_carton", (201, 190, 157), (92, 124, 79), "FARM EGGS", "refrigerated", parts=("tray", "hinged_lid", "six_domes", "latch")),
    _spec("butter_pack", "refrigerated", (0.16, 0.065, 0.055), "butter_pack", (239, 221, 123), (56, 108, 169), "CREAMERY BUTTER", "refrigerated", parts=("wrapped_block", "folded_ends", "paper_band")),
    _spec("cheese_wedge", "refrigerated", (0.15, 0.085, 0.075), "cheese_wedge", (236, 185, 57), (130, 58, 43), "AGED CHEDDAR", "refrigerated", parts=("triangular_wedge", "wax_rind", "label_panel")),
    _spec("frozen_pizza", "frozen", (0.29, 0.045, 0.29), "flat_carton", (185, 62, 44), (238, 193, 72), "STONE PIZZA", "refrigerated", parts=("shallow_carton", "edge_lip", "round_window")),
    _spec("icecream_tub", "frozen", (0.13, 0.11, 0.15), "oval_tub", (89, 142, 184), (237, 197, 119), "VANILLA CREAM", "refrigerated", parts=("oval_tub", "rolled_rim", "domed_lid")),
    _spec("olive_oil", "condiments", (0.085, 0.075, 0.29), "oil_bottle", (91, 118, 55), (224, 185, 72), "GROVE OIL", parts=("square_glass_body", "sloped_shoulders", "long_neck", "pour_spout", "cap")),
    _spec("maple_syrup", "condiments", (0.12, 0.075, 0.23), "handled_bottle", (154, 79, 40), (232, 166, 63), "MAPLE SYRUP", parts=("flask_body", "neck", "side_handle", "cap")),
    _spec("sports_drink", "beverage", (0.095, 0.095, 0.245), "grip_bottle", (66, 147, 190), (223, 230, 86), "ACTIVE HYDRATE", "beverage", parts=("ribbed_body", "waist", "shoulder", "sports_cap")),
    _spec("sparkling_wine", "beverage", (0.085, 0.085, 0.31), "longneck_bottle", (58, 101, 76), (224, 192, 116), "ORCHARD SPARKLE", "beverage", parts=("glass_body", "deep_punt", "long_shoulders", "foil_neck", "cork")),
    _spec("spray_cleaner", "cleaning", (0.115, 0.075, 0.29), "trigger_spray", (62, 162, 172), (232, 224, 82), "CLEAR SURFACE", "household", parts=("bottle_body", "offset_neck", "trigger_head", "nozzle", "lever")),
    _spec("dish_soap", "cleaning", (0.09, 0.06, 0.245), "squeeze_bottle", (71, 161, 105), (231, 211, 75), "CITRUS DISH", "household", parts=("waisted_body", "shoulder", "flip_spout")),
    _spec("detergent_jug", "cleaning", (0.20, 0.12, 0.31), "detergent_jug", (46, 103, 167), (230, 149, 52), "FRESH LAUNDRY", "household", parts=("broad_jug", "hollow_handle", "angled_spout", "measuring_cap")),
    _spec("bleach_jug", "cleaning", (0.17, 0.115, 0.295), "bleach_jug", (230, 235, 229), (78, 113, 175), "BRIGHT WHITE", "household", parts=("jug_body", "high_handle", "offset_spout", "safety_cap")),
    _spec("hand_soap", "cleaning", (0.085, 0.07, 0.20), "pump_bottle", (207, 144, 178), (246, 226, 190), "GENTLE HANDS", "household", parts=("rounded_body", "neck", "pump_stem", "pump_head", "nozzle")),
    _spec("shampoo", "cleaning", (0.09, 0.065, 0.24), "flip_bottle", (93, 78, 152), (223, 186, 78), "DAILY SHAMPOO", "household", parts=("tapered_body", "recessed_grip", "bottom_flip_cap")),
    _spec("tuna_can", "cans", (0.095, 0.095, 0.045), "short_can", (77, 124, 166), (225, 197, 77), "OCEAN TUNA", parts=("short_can", "double_rim", "pull_tab")),
    _spec("coffee_canister", "cans", (0.13, 0.13, 0.19), "canister", (105, 56, 39), (225, 174, 62), "ROAST COFFEE", parts=("wide_canister", "rolled_base", "overcap", "grip_ribs")),
    _spec("sardine_tin", "cans", (0.115, 0.075, 0.035), "rectangular_tin", (57, 118, 151), (229, 200, 86), "COAST SARDINES", parts=("rounded_tin", "rolled_seam", "key_tab")),
    _spec("tea_box", "pantry_box", (0.145, 0.075, 0.095), "hinged_box", (78, 132, 87), (229, 196, 84), "GARDEN TEA", parts=("low_box", "hinged_lid", "front_flap")),
    _spec("pasta_box", "pantry_box", (0.085, 0.06, 0.275), "window_box", (54, 112, 167), (231, 191, 75), "BRONZE PASTA", parts=("tall_carton", "cellophane_window", "top_flaps")),
    _spec("rice_bag", "bagged_goods", (0.145, 0.085, 0.25), "gusset_bag", (225, 217, 183), (68, 121, 91), "LONG GRAIN RICE", parts=("gusseted_sack", "folded_base", "sealed_top", "side_panels")),
    _spec("coffee_bag", "bagged_goods", (0.135, 0.08, 0.235), "valve_bag", (76, 52, 42), (210, 145, 55), "HOUSE COFFEE", parts=("flat_bottom_pouch", "sealed_top", "degassing_valve")),
    _spec("flour_sack", "bagged_goods", (0.14, 0.085, 0.235), "paper_sack", (228, 218, 190), (158, 61, 45), "BAKER FLOUR", parts=("paper_sack", "pinched_top", "folded_base", "side_gussets")),
    _spec("bread_loaf", "bakery", (0.24, 0.105, 0.13), "bread_bag", (208, 144, 65), (79, 121, 168), "GRAIN LOAF", "bakery", parts=("continuous_scored_loaf", "clear_sleeve", "bottom_seal", "twisted_neck", "closure")),
    _spec("chips_bag", "snack_bag", (0.155, 0.07, 0.255), "pillow_bag", (186, 62, 50), (234, 196, 68), "KETTLE CHIPS", parts=("inflated_pouch", "top_crimp", "bottom_crimp", "side_seams")),
    _spec("banana_bunch", "fresh_produce", (0.19, 0.105, 0.105), "banana_bunch", (225, 192, 55), (95, 121, 54), "BANANA BUNCH", "produce", parts=("five_continuous_tapered_fingers", "crown", "stem")),
    _spec("pear", "fresh_produce", (0.095, 0.09, 0.13), "pear", (149, 177, 66), (111, 76, 40), "GREEN PEAR", "produce", parts=("bulb", "tapered_top", "stem", "leaf")),
    _spec("broccoli", "fresh_produce", (0.14, 0.12, 0.15), "broccoli", (53, 112, 60), (117, 157, 74), "BROCCOLI", "produce", parts=("stalk", "five_tapered_branches", "fifteen_irregular_floret_meshes")),
    _spec("carrot_bunch", "fresh_produce", (0.14, 0.08, 0.22), "carrot_bunch", (225, 113, 37), (63, 126, 61), "CARROT BUNCH", "produce", parts=("three_tapered_roots", "binding_band", "leaf_cluster")),
    _spec("jam_jar", "jars", (0.085, 0.085, 0.12), "jam_jar", (154, 46, 67), (232, 191, 87), "BERRY JAM", parts=("faceted_glass", "fruit_fill", "twist_lid", "neck_label")),
    _spec("spice_jar", "jars", (0.055, 0.055, 0.12), "spice_jar", (171, 89, 42), (237, 206, 96), "SMOKED PAPRIKA", parts=("small_jar", "shaker_insert", "ribbed_cap")),
    _spec("peanut_jar", "jars", (0.12, 0.10, 0.16), "wide_jar", (184, 122, 48), (72, 109, 165), "PEANUT SPREAD", parts=("wide_body", "shoulder", "recessed_grips", "broad_lid")),
    _spec("paper_towel", "household", (0.145, 0.145, 0.28), "paper_roll", (235, 235, 225), (68, 135, 171), "PAPER TOWELS", "household", parts=("paper_cylinder", "cardboard_core", "embossed_bands", "wrapper")),
    _spec("tissue_box", "household", (0.225, 0.115, 0.095), "tissue_box", (81, 151, 174), (226, 211, 103), "SOFT TISSUES", "household", parts=("carton", "oval_opening", "raised_tissue")),
    _spec("foil_box", "household", (0.31, 0.055, 0.055), "long_box", (73, 117, 163), (211, 213, 210), "KITCHEN FOIL", "household", parts=("long_carton", "hinged_flap", "serrated_edge", "foil_roll")),
    _spec("sponge_pack", "household", (0.15, 0.045, 0.19), "blister_pack", (226, 189, 61), (70, 135, 87), "SCRUB SPONGES", "household", parts=("card_backer", "clear_blister", "two_sponges", "hang_slot")),
    _spec("trash_bags", "household", (0.19, 0.105, 0.22), "handled_box", (55, 73, 84), (86, 153, 171), "STRONG BAGS", "household", parts=("carton", "carry_handle", "dispensing_slot")),
    _spec("angled_produce_bin", "produce_fixture", (0.46, 0.36, 0.24), "angled_bin", (112, 87, 59), (154, 119, 69), "PRODUCE BIN", "produce", "shelf", ("sloped_base", "low_front", "high_back", "side_cheeks", "rails")),
    _spec("wicker_basket", "produce_fixture", (0.42, 0.31, 0.20), "wicker_basket", (142, 101, 58), (191, 145, 81), "MARKET BASKET", "produce", "shelf", ("woven_base", "four_posts", "sixteen_extruded_diagonal_ribbons", "four_cross_slats", "rim")),
    _spec("shelf_divider", "shelf_fixture", (0.012, 0.42, 0.14), "shelf_divider", (205, 211, 209), (92, 139, 166), "SHELF DIVIDER", "fixtures", "shelf", ("base_clip", "vertical_fin", "front_stop")),
    _spec("bottle_rack", "shelf_fixture", (0.34, 0.42, 0.07), "bottle_rack", (90, 96, 98), (171, 178, 180), "BOTTLE RACK", "fixtures", "shelf", ("base", "four_channels", "front_stops", "rear_stop")),
    _spec("price_display", "shelf_fixture", (0.26, 0.018, 0.065), "price_display", (48, 55, 59), (221, 224, 214), "PRICE DISPLAY", "fixtures", "shelf_edge", ("rail_clip", "eink_panel", "bezel")),
)


# Explicit art direction prevents the new range from reading as one house
# brand with recolors.  The final integer selects a materially different label
# composition, while the first two strings are item-specific brand and copy.
ITEM_ART_DIRECTIONS: dict[str, tuple[str, str, int]] = {
    "milk_gallon": ("BLUEBELL DAIRY", "WHOLE MILK • KEEP COLD", 0),
    "milk_carton": ("VALLEY FOLD", "PASTURE MILK", 3),
    "yogurt_cup": ("SPOON & BERRY", "CULTURED • LIVE", 4),
    "egg_carton": ("SIX OAK FARM", "GRADE A • LARGE", 2),
    "butter_pack": ("GOLD CHURN", "SALTED CREAMERY BUTTER", 5),
    "cheese_wedge": ("CELLAR WHEEL", "AGED 14 MONTHS", 1),
    "frozen_pizza": ("EMBER STONE", "WOOD-FIRED STYLE", 3),
    "icecream_tub": ("SNOWCAP CREAM", "VANILLA BEAN", 0),
    "olive_oil": ("VERDE GROVE", "FIRST COLD PRESS", 1),
    "maple_syrup": ("NORTH MAPLE", "DARK ROBUST", 5),
    "sports_drink": ("PULSE RUN", "ELECTROLYTE BLEND", 2),
    "sparkling_wine": ("ORCHARD NO. 7", "DRY APPLE SPARKLE", 4),
    "spray_cleaner": ("CLEARLINE", "KITCHEN + GLASS", 0),
    "dish_soap": ("LEMON DROP", "CUTS GREASE", 3),
    "detergent_jug": ("TIDEMARK", "32 LAUNDRY LOADS", 1),
    "bleach_jug": ("WHITE SHIELD", "CONCENTRATED", 5),
    "hand_soap": ("SOFT PALM", "ALOE HAND WASH", 4),
    "shampoo": ("RINSE DAY", "DAILY BALANCE", 2),
    "tuna_can": ("BLUE PIER", "SOLID LIGHT TUNA", 1),
    "coffee_canister": ("ROASTER 8", "DARK ROAST", 3),
    "sardine_tin": ("PORT & NET", "OLIVE OIL PACK", 5),
    "tea_box": ("GREENHOUSE TEA", "GARDEN HERBAL", 0),
    "pasta_box": ("BRONZE DIE", "RIGATONI NO. 42", 2),
    "rice_bag": ("RIVER PADDY", "LONG GRAIN", 4),
    "coffee_bag": ("NIGHT OWL", "WHOLE BEAN", 1),
    "flour_sack": ("RED MILLSTONE", "UNBLEACHED FLOUR", 3),
    "bread_loaf": ("MORNING SLICE", "SEEDED WHEAT", 5),
    "chips_bag": ("KETTLE YARD", "SEA SALT CRUNCH", 0),
    "banana_bunch": ("SUN ARC", "FARM LOT 24", 2),
    "pear": ("HILLSIDE PEAR", "ORCHARD SELECT", 4),
    "broccoli": ("GREEN CROWN", "FIELD CUT", 1),
    "carrot_bunch": ("ROOT ROW", "BUNCHED TODAY", 3),
    "jam_jar": ("BERRY PATCH", "SMALL-BATCH JAM", 5),
    "spice_jar": ("TIN SPOON", "SMOKED PAPRIKA", 0),
    "peanut_jar": ("NUT HOUSE", "CRUNCHY SPREAD", 2),
    "paper_towel": ("DRYDOCK", "2-PLY ABSORBENT", 4),
    "tissue_box": ("CLOUD SOFT", "FACIAL TISSUE", 1),
    "foil_box": ("SILVER EDGE", "HEAVY-DUTY FOIL", 3),
    "sponge_pack": ("SCRUB CREW", "NON-SCRATCH DUO", 5),
    "trash_bags": ("CARRY STRONG", "DRAWSTRING • 30 GAL", 0),
    "angled_produce_bin": ("HARVEST RACK", "SLOPED DISPLAY BIN", 2),
    "wicker_basket": ("MARKET WEAVE", "HANDLED DISPLAY", 4),
    "shelf_divider": ("AISLEFORM", "CLEAR DIVIDER", 1),
    "bottle_rack": ("ROLLFORWARD", "4-LANE RACK", 3),
    "price_display": ("INKTAG 260", "SHELF PRICE", 5),
}


# These eight packages dominate the representative 9 s/17 s RGB views. They
# use a complete fictional wrap rather than a repeated square label: side copy,
# a distinct back panel, and the food illustration all remain deterministic and
# project-authored. The tuple is brand, flavor/variety copy, factual secondary
# copy, and the procedural illustration family.
HERO_ART_DIRECTIONS: dict[str, tuple[str, str, str, str]] = {
    "cereal_sunrise": ("DAWNFIELD", "ROLLED OATS + RED BERRIES", "WHOLE GRAIN • 12 SERVINGS", "oat_bowl"),
    "cereal_harvest": ("RIVERBEND", "TOASTED GRAIN LOOPS", "FAMILY SIZE • WHOLE GRAIN", "grain_loops"),
    "cereal_grain": ("FIELD NOTES", "SEVEN-GRAIN FLAKES", "LIGHTLY TOASTED • HIGH FIBER", "wheat"),
    "cereal_berry": ("BLUE HILL", "BERRY OAT CLUSTERS", "DRIED BERRIES • CRISP OATS", "berries"),
    "cereal_honey": ("GOLDEN COMB", "HONEY ALMOND CRUNCH", "PURE HONEY • ROASTED NUTS", "honey"),
    "cereal_morning": ("FIRST LIGHT", "FRUIT + GRAIN MIX", "RAISINS • OATS • SEEDS", "muesli"),
    "juice_citrus": ("SUN ORCHARD", "ORANGE + TANGERINE", "NOT FROM CONCENTRATE • 1 L", "citrus"),
    "coffee_bag": ("NIGHT OWL", "SUMATRA DARK ROAST", "WHOLE BEAN • ROAST 04", "coffee"),
}

HERO_FOOD_SOURCES: dict[str, tuple[str, str]] = {
    "cereal_sunrise": (
        "oat_berry_bowl_v1.png",
        "05092075da2683fdb96bc5ade5e8173813ff52a5837cf72f180100dd982b2139",
    ),
    "cereal_harvest": (
        "toasted_loops_bowl_v1.png",
        "43e5d2f9abae2f9389c5f4af596fb82263df19a0e81c65ba1e64c38aefefe70f",
    ),
    "juice_citrus": (
        "citrus_still_life_v1.png",
        "ba636324e9fff4e56cc35738206fcb72f8f54e5d0ec76827ebafaf03cf1df86b",
    ),
    "coffee_bag": (
        "roasted_coffee_scoop_v1.png",
        "e7cb189a27ed377419a1ed492d959dd8f8aa8b2d3c45334e76490ac98c1dab56",
    ),
}

BOTTLE_LEFT_STRIP_FRACTION = 0.16
BOTTLE_LAYOUT_FIVE_TEXT_INSET_FRACTION = 0.20


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def _write_png(path: Path, width: int, height: int, pixels: list[list[tuple[int, int, int]]]) -> None:
    raw = b"".join(b"\x00" + bytes(channel for pixel in row for channel in pixel) for row in pixels)
    payload = b"\x89PNG\r\n\x1a\n"
    payload += _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    payload += _png_chunk(b"IDAT", zlib.compress(raw, 9))
    payload += _png_chunk(b"IEND", b"")
    path.write_bytes(payload)


def _blend(a: tuple[int, int, int], b: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    return tuple(int(round(x * (1.0 - amount) + y * amount)) for x, y in zip(a, b))


def _font(size: int, bold: bool = False):
    path, _ = REQUIRED_FONTS[bold]
    return ImageFont.truetype(path, size)


def _write_rich_texture(path: Path, spec: AssetSpec, size: int = 768) -> None:
    """Write a dense, readable fictional package face with no trademarked art."""
    if Image is None:
        raise RuntimeError(
            f"Pillow=={REQUIRED_PILLOW_VERSION} is required; refusing to write fallback texture {path}"
        )
    base = tuple(int(round(v * 255)) for v in spec.color)
    accent = tuple(int(round(v * 255)) for v in spec.accent)
    dark = _blend(base, (12, 18, 24), 0.62)
    light = _blend(base, (255, 255, 255), 0.28)
    image = Image.new("RGB", (size, size), base)
    draw = ImageDraw.Draw(image)
    # Directional print-like gradient and restrained paper grain.
    for y in range(size):
        t = y / max(1, size - 1)
        row = _blend(light, base, t)
        draw.line((0, y, size, y), fill=row)
    for x in range(0, size, 24):
        draw.line((x, 0, x + size // 4, size), fill=_blend(base, accent, 0.16), width=3)
    # Brand band and a contrasting product ribbon.
    band_h = int(size * 0.16)
    draw.rounded_rectangle((int(size * 0.055), int(size * 0.045), int(size * 0.945), band_h), radius=22, fill=dark)
    draw.text((int(size * 0.09), int(size * 0.068)), "NORTHSTAR MARKET", font=_font(int(size * 0.055), True), fill=(245, 244, 232))
    ribbon_y = int(size * 0.77)
    draw.rectangle((0, ribbon_y, size, int(size * 0.88)), fill=accent)
    draw.text((int(size * 0.08), ribbon_y + int(size * 0.025)), "SMALL-BATCH • EVERYDAY GOOD", font=_font(int(size * 0.026), True), fill=dark)
    # Product badge and category-specific graphic accents.
    cx, cy = int(size * 0.72), int(size * 0.42)
    radius = int(size * 0.155)
    draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), fill=accent, outline=(255, 255, 255), width=7)
    draw.ellipse((cx - int(radius * .55), cy - int(radius * .55), cx + int(radius * .55), cy + int(radius * .55)), outline=light, width=5)
    if spec.category in {"cereal", "snacks"}:
        for dx, dy, rr in ((-70, 40, 25), (-18, 58, 31), (42, 42, 22), (84, 62, 28)):
            draw.ellipse((cx + dx - rr, cy + dy - rr, cx + dx + rr, cy + dy + rr), fill=_blend(accent, (255, 255, 255), .25), outline=dark, width=3)
    elif spec.category in {"juice", "water", "soda"}:
        draw.arc((cx - 80, cy - 82, cx + 80, cy + 82), 200, 340, fill=(255, 255, 255), width=12)
        draw.line((cx - 55, cy + 40, cx + 65, cy - 46), fill=(255, 255, 255), width=10)
    elif spec.category in {"cans", "jars"}:
        draw.rectangle((cx - 66, cy - 42, cx + 66, cy + 44), outline=(255, 255, 255), width=7)
        draw.line((cx - 42, cy, cx + 42, cy), fill=(255, 255, 255), width=8)
    else:
        draw.arc((cx - 76, cy - 76, cx + 76, cy + 76), 25, 315, fill=(255, 255, 255), width=10)
    # Product title hierarchy.
    title_font = _font(int(size * (0.095 if len(spec.product_name) < 13 else 0.073)), True)
    subtitle = {
        "cereal": "WHOLE GRAIN • BREAKFAST",
        "snacks": "CRISP • SHAREABLE",
        "juice": "FRUIT BLEND • NO FUSS",
        "water": "SPRING WATER • 500 ml",
        "soda": "SPARKLING BOTANICAL",
        "cans": "PANTRY CLASSIC",
        "jars": "SMALL BATCH • PANTRY",
        "red_apples": "ORCHARD SELECT",
        "green_apples": "ORCHARD SELECT",
        "oranges": "CITRUS SELECT",
        "lemons": "CITRUS SELECT",
        "produce_crate": "FARM STAND",
        "promotional_sign": "FRESH MARKET",
    }.get(spec.category, "QUALITY GROCERIES")
    draw.text((int(size * 0.08), int(size * 0.31)), spec.product_name, font=title_font, fill=(255, 255, 255), stroke_width=3, stroke_fill=dark)
    draw.text((int(size * 0.09), int(size * 0.56)), subtitle, font=_font(int(size * 0.033), True), fill=dark)
    # Side nutrition panel and real-looking machine-readable mark.
    panel_x = int(size * 0.08)
    panel_y = int(size * 0.62)
    draw.rounded_rectangle((panel_x, panel_y, int(size * 0.47), panel_y + int(size * 0.105)), radius=10, fill=_blend(base, (255, 255, 255), .72))
    for row in range(4):
        yy = panel_y + int(size * 0.018) + row * int(size * 0.021)
        draw.line((panel_x + int(size * .025), yy, panel_x + int(size * .34), yy), fill=dark, width=3 if row == 0 else 2)
    barcode_x = int(size * 0.73)
    barcode_y = int(size * 0.895)
    for index in range(22):
        width = 3 if index % 5 == 0 else 1 + (index % 3)
        draw.rectangle((barcode_x + index * 7, barcode_y, barcode_x + index * 7 + width, barcode_y + int(size * .075)), fill=dark)
    draw.text((int(size * 0.08), int(size * 0.91)), "FICTIONAL PRODUCT • 100% DEMO", font=_font(int(size * .022)), fill=dark)
    image.save(path, format="PNG", optimize=True)


def _hero_wrap_regions(spec: AssetSpec) -> dict[str, tuple[float, float, float, float]]:
    """Return UV regions for a conventional left/front/right/back package wrap."""
    width, depth, _ = spec.dimensions_m
    perimeter = 2.0 * (width + depth)
    left_end = depth / perimeter
    front_end = (depth + width) / perimeter
    right_end = (2.0 * depth + width) / perimeter
    return {
        "left": (0.0, 0.04, left_end, 0.96),
        "front": (left_end, 0.04, front_end, 0.96),
        "right": (front_end, 0.04, right_end, 0.96),
        "back": (right_end, 0.04, 1.0, 0.96),
    }


def _draw_food_illustration(
    draw,
    kind: str,
    box: tuple[int, int, int, int],
    ink: tuple[int, int, int],
    accent: tuple[int, int, int],
    seed: bytes,
) -> None:
    """Layer shaded shapes into item-specific, food-readable artwork."""
    x0, y0, x1, y1 = box
    width, height = x1 - x0, y1 - y0
    cx, cy = x0 + width // 2, y0 + int(height * .58)
    shadow = _blend(ink, (0, 0, 0), .34)
    cream = (242, 226, 188)

    if kind in {"oat_bowl", "grain_loops", "berries", "muesli"}:
        draw.ellipse((x0 + int(width * .10), y0 + int(height * .48), x1 - int(width * .06), y1 - int(height * .03)), fill=shadow)
        draw.pieslice((x0 + int(width * .08), y0 + int(height * .32), x1 - int(width * .04), y1 - int(height * .04)), 0, 180, fill=(231, 225, 207), outline=ink, width=max(2, width // 90))
        draw.ellipse((x0 + int(width * .08), y0 + int(height * .31), x1 - int(width * .04), y0 + int(height * .62)), fill=(248, 240, 218), outline=ink, width=max(2, width // 100))
        for index in range(18):
            px = x0 + int(width * (.16 + .66 * ((seed[index % 32] + index * 29) % 101) / 100.0))
            py = y0 + int(height * (.37 + .17 * ((seed[(index + 7) % 32] + index * 11) % 101) / 100.0))
            radius = max(3, int(width * (.022 + .010 * (index % 3))))
            if kind == "grain_loops":
                draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=(205, 137, 63), outline=shadow, width=max(1, radius // 4))
                inner = max(2, radius // 2)
                draw.ellipse((px - inner, py - inner, px + inner, py + inner), fill=(245, 222, 174))
            elif kind in {"berries", "oat_bowl"} and index % 4 == 0:
                berry = (137, 35, 55) if index % 8 == 0 else (62, 72, 124)
                draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=berry, outline=shadow)
                draw.ellipse((px - radius // 3, py - radius // 2, px, py - radius // 6), fill=(246, 213, 196))
            else:
                draw.ellipse((px - radius, py - radius // 2, px + radius, py + radius // 2), fill=(194, 145, 72), outline=shadow)
        draw.arc((x0 + int(width * .12), y0 + int(height * .39), x1 - int(width * .08), y0 + int(height * .66)), 190, 345, fill=(255, 255, 248), width=max(2, width // 42))
    elif kind == "wheat":
        for stalk in range(5):
            sx = x0 + int(width * (.22 + stalk * .13))
            top = y0 + int(height * (.20 + .035 * (stalk % 2)))
            draw.line((sx, y1 - int(height * .06), sx + int(width * .05), top), fill=shadow, width=max(3, width // 70))
            for grain in range(6):
                gy = top + int(grain * height * .075)
                gx = sx + int(width * (.05 - grain * .006))
                direction = -1 if grain % 2 else 1
                grain_end = gx + direction * int(width * .10)
                draw.ellipse((min(gx, grain_end), gy, max(gx, grain_end), gy + int(height * .055)), fill=(218, 164, 71), outline=shadow)
    elif kind == "honey":
        radius = min(width, height) // 7
        for row in range(3):
            for col in range(3):
                px = x0 + int(width * (.22 + col * .24 + .12 * (row % 2)))
                py = y0 + int(height * (.28 + row * .20))
                draw.regular_polygon((px, py, radius), 6, rotation=30, fill=(226, 163, 41), outline=shadow)
                draw.regular_polygon((px, py, max(2, radius - width // 32)), 6, rotation=30, fill=(248, 194, 63))
        draw.line((x0 + int(width * .12), y0 + int(height * .13), x1 - int(width * .05), y0 + int(height * .70)), fill=(113, 69, 33), width=max(5, width // 30))
        draw.rounded_rectangle((cx - width // 7, cy - height // 10, cx + width // 7, cy), radius=max(3, width // 25), fill=(190, 122, 37), outline=shadow)
    elif kind == "citrus":
        for index, (dx, dy, scale) in enumerate(((-.20, .08, .32), (.12, -.06, .38), (.25, .22, .25))):
            radius = int(min(width, height) * scale)
            px, py = cx + int(width * dx), cy + int(height * dy)
            outer = (235, 129, 28) if index != 2 else (239, 173, 34)
            draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=outer, outline=shadow, width=max(2, width // 80))
            draw.ellipse((px - int(radius * .82), py - int(radius * .82), px + int(radius * .82), py + int(radius * .82)), fill=(255, 220, 116), outline=cream, width=max(2, width // 65))
            for spoke in range(8):
                angle = spoke * math.pi / 4.0
                draw.line((px, py, px + int(math.cos(angle) * radius * .78), py + int(math.sin(angle) * radius * .78)), fill=cream, width=max(1, width // 100))
        draw.ellipse((x0 + int(width * .15), y0 + int(height * .10), x0 + int(width * .52), y0 + int(height * .26)), fill=(72, 128, 68), outline=shadow)
    else:  # coffee
        draw.ellipse((x0 + int(width * .11), y0 + int(height * .22), x1 - int(width * .08), y1 - int(height * .04)), fill=(44, 29, 23), outline=(15, 12, 10), width=max(3, width // 75))
        for index in range(13):
            px = x0 + int(width * (.18 + .66 * ((seed[index] + index * 17) % 101) / 100.0))
            py = y0 + int(height * (.30 + .48 * ((seed[(index + 11) % 32] + index * 23) % 101) / 100.0))
            rx, ry = max(5, width // 18), max(7, height // 16)
            draw.ellipse((px - rx, py - ry, px + rx, py + ry), fill=(111, 66, 41), outline=(31, 20, 16), width=max(2, width // 120))
            draw.arc((px - rx // 2, py - ry, px + rx // 2, py + ry), 80, 280, fill=(221, 158, 102), width=max(1, width // 140))
        draw.arc((x0 + int(width * .40), y0 + int(height * .04), x1 - int(width * .18), y0 + int(height * .36)), 190, 335, fill=(235, 226, 207), width=max(2, width // 55))


def _paste_hero_food_source(image, spec: AssetSpec, box: tuple[int, int, int, int]) -> bool:
    """Composite one pinned original RGBA food cutout; return false for vector-only art."""
    source_record = HERO_FOOD_SOURCES.get(spec.asset_key)
    if source_record is None:
        return False
    filename, expected_hash = source_record
    source_path = Path(__file__).resolve().parents[2] / "assets" / "retail" / "source_food" / filename
    if not source_path.is_file():
        raise RuntimeError(f"required hero food source missing: {source_path}")
    actual_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
    if actual_hash != expected_hash:
        raise RuntimeError(
            f"hero food source hash mismatch: {source_path} ({actual_hash}; expected {expected_hash})"
        )
    with Image.open(source_path) as opened:
        source = opened.convert("RGBA")
    alpha_bounds = source.getchannel("A").getbbox()
    if alpha_bounds is None:
        raise RuntimeError(f"hero food source has no visible alpha content: {source_path}")
    source = source.crop(alpha_bounds)
    x0, y0, x1, y1 = box
    target_width, target_height = x1 - x0, y1 - y0
    source.thumbnail((target_width, target_height), Image.Resampling.LANCZOS)
    paste_x = x0 + (target_width - source.width) // 2
    paste_y = y0 + (target_height - source.height) // 2
    image.paste(source, (paste_x, paste_y), source)
    return True


def _write_hero_wrap_texture(path: Path, spec: AssetSpec, height: int = 768) -> None:
    """Write a four-panel retail wrap for packages closest to the RGB camera."""
    if Image is None:
        raise RuntimeError(f"Pillow=={REQUIRED_PILLOW_VERSION} is required; refusing to write {path}")
    regions = _hero_wrap_regions(spec)
    physical_width, physical_depth, physical_height = spec.dimensions_m
    width = int(round(height * 2.0 * (physical_width + physical_depth) / physical_height))
    width = max(960, min(1536, width))
    base = tuple(int(round(value * 255)) for value in spec.color)
    accent = tuple(int(round(value * 255)) for value in spec.accent)
    ink = _blend(base, (8, 12, 16), .78)
    paper = _blend(base, (250, 246, 231), .66)
    seed = hashlib.sha256((spec.asset_key + ":hero-wrap-v1").encode("utf-8")).digest()
    image = Image.new("RGB", (width, height), paper)
    draw = ImageDraw.Draw(image)

    for y in range(height):
        amount = .06 + .11 * y / max(1, height - 1)
        draw.line((0, y, width, y), fill=_blend(paper, base, amount))
    for index in range(420):
        x = (seed[index % 32] * (index + 19) + index * 37) % width
        y = (seed[(index + 9) % 32] * (index + 31) + index * 17) % height
        dot = _blend(paper, ink, .07 + .03 * (index % 3))
        draw.point((x, y), fill=dot)

    px = {
        name: (int(round(bounds[0] * width)), int(round(bounds[1] * height)), int(round(bounds[2] * width)), int(round(bounds[3] * height)))
        for name, bounds in regions.items()
    }
    brand, variety, secondary, illustration = HERO_ART_DIRECTIONS[spec.asset_key]
    lx0, ly0, lx1, ly1 = px["left"]
    fx0, fy0, fx1, fy1 = px["front"]
    rx0, ry0, rx1, ry1 = px["right"]
    bx0, by0, bx1, by1 = px["back"]
    front_width = fx1 - fx0

    draw.rectangle((lx0, ly0, lx1, ly1), fill=_blend(base, paper, .36), outline=ink, width=max(2, width // 360))
    draw.rectangle((lx0, ly0, lx1, ly0 + int((ly1 - ly0) * .18)), fill=ink)
    draw.text(((lx0 + lx1) // 2, ly0 + int(height * .035)), brand, font=_font(max(13, int((lx1 - lx0) * .095)), True), fill=(246, 244, 231), anchor="ma")
    draw.text((lx0 + int((lx1 - lx0) * .10), ly0 + int(height * .23)), "INGREDIENTS", font=_font(max(13, int((lx1 - lx0) * .085)), True), fill=ink)
    for row, scale in enumerate((.76, .88, .64, .82, .70, .55)):
        y = ly0 + int(height * (.30 + row * .055))
        draw.line((lx0 + int((lx1 - lx0) * .10), y, lx0 + int((lx1 - lx0) * scale), y), fill=ink, width=max(2, height // 320))
    draw.text(((lx0 + lx1) // 2, ly1 - int(height * .09)), "PACKED FOR\nNORTHSTAR MARKET", font=_font(max(12, int((lx1 - lx0) * .068)), True), fill=ink, anchor="mm", align="center")

    draw.rectangle((rx0, ry0, rx1, ry1), fill=_blend(accent, paper, .40), outline=ink, width=max(2, width // 360))
    draw.text(((rx0 + rx1) // 2, ry0 + int(height * .10)), "SERVING\nSUGGESTION", font=_font(max(13, int((rx1 - rx0) * .08)), True), fill=ink, anchor="mm", align="center")
    for row in range(5):
        y = ry0 + int(height * (.25 + row * .095))
        draw.rectangle((rx0 + int((rx1 - rx0) * .12), y, rx1 - int((rx1 - rx0) * .12), y + max(2, height // 145)), fill=_blend(ink, accent, .18 * (row % 2)))
    draw.text(((rx0 + rx1) // 2, ry1 - int(height * .10)), "NET WT\nFICTIONAL PRODUCT", font=_font(max(12, int((rx1 - rx0) * .07)), True), fill=ink, anchor="mm", align="center")

    draw.rectangle((bx0, by0, bx1, by1), fill=(244, 241, 225), outline=ink, width=max(2, width // 360))
    back_margin = max(10, int((bx1 - bx0) * .07))
    draw.text((bx0 + back_margin, by0 + int(height * .035)), "Nutrition Facts", font=_font(max(18, int((bx1 - bx0) * .075)), True), fill=(16, 16, 14))
    rule_y = by0 + int(height * .10)
    draw.rectangle((bx0 + back_margin, rule_y, bx1 - back_margin, rule_y + max(5, height // 95)), fill=(18, 18, 16))
    for row in range(9):
        y = rule_y + int(height * (.045 + row * .054))
        draw.line((bx0 + back_margin, y, bx1 - back_margin, y), fill=(28, 28, 25), width=2 if row not in {2, 6} else 5)
    barcode_left = bx0 + back_margin
    barcode_top = by1 - int(height * .20)
    cursor = barcode_left
    for index in range(31):
        bar_width = 1 + seed[index % 32] % 4
        draw.rectangle((cursor, barcode_top, cursor + bar_width, by1 - int(height * .06)), fill=(20, 20, 18))
        cursor += bar_width + 2 + seed[(index + 13) % 32] % 3
    draw.text((bx1 - back_margin, by1 - int(height * .10)), "LOT " + spec.asset_key[-4:].upper(), font=_font(max(12, int((bx1 - bx0) * .045))), fill=ink, anchor="rm")

    draw.rounded_rectangle((fx0, fy0, fx1, fy1), radius=max(10, front_width // 24), fill=_blend(base, paper, .18), outline=ink, width=max(3, front_width // 75))
    draw.rectangle((fx0, fy0, fx1, fy0 + int(height * .13)), fill=ink)
    draw.text(((fx0 + fx1) // 2, fy0 + int(height * .062)), brand, font=_font(max(18, int(front_width * .085)), True), fill=(247, 244, 230), anchor="mm")
    draw.text(((fx0 + fx1) // 2, fy0 + int(height * .205)), spec.product_name, font=_font(max(25, int(front_width * (.115 if len(spec.product_name) < 13 else .092))), True), fill=ink, anchor="mm")
    draw.text(((fx0 + fx1) // 2, fy0 + int(height * .275)), variety, font=_font(max(14, int(front_width * .048)), True), fill=_blend(ink, accent, .18), anchor="mm")
    art_box = (fx0 + int(front_width * .08), fy0 + int(height * .30), fx1 - int(front_width * .08), fy0 + int(height * .76))
    if not _paste_hero_food_source(image, spec, art_box):
        _draw_food_illustration(draw, illustration, art_box, ink, accent, seed)
    draw.rectangle((fx0, fy0 + int(height * .78), fx1, fy1), fill=accent)
    draw.text(((fx0 + fx1) // 2, fy0 + int(height * .835)), secondary, font=_font(max(13, int(front_width * .047)), True), fill=ink, anchor="mm")
    draw.text(((fx0 + fx1) // 2, fy0 + int(height * .895)), "ORIGINAL FICTIONAL PACKAGING • DEMO", font=_font(max(11, int(front_width * .034))), fill=ink, anchor="mm")
    image.save(path, format="PNG", optimize=True)


def _new_art_system(spec: AssetSpec) -> str:
    if spec.department == "fixtures":
        return "fixture"
    if spec.department == "produce":
        return "produce"
    if spec.department == "household":
        return "household"
    if spec.department == "refrigerated":
        return "chilled"
    if spec.department == "bakery":
        return "bakery"
    if spec.model_type in {
        "oil_bottle", "handled_bottle", "grip_bottle", "longneck_bottle",
        "squeeze_bottle", "flip_bottle", "pump_bottle", "trigger_spray",
        "detergent_jug", "bleach_jug",
    }:
        return "bottle"
    if spec.model_type in {"short_can", "rectangular_tin", "canister", "jam_jar", "wide_jar", "spice_jar"}:
        return "wrap"
    if spec.model_type in {"gusset_bag", "valve_bag", "paper_sack", "pillow_bag", "bread_bag"}:
        return "soft_pack"
    return "carton"


def _new_artwork_size(spec: AssetSpec) -> tuple[int, int]:
    """Match bitmap aspect to its physical label instead of stretching a square."""
    if spec.department == "fixtures":
        return (768, 256)
    width, _, height = spec.dimensions_m
    ratio = max(0.50, min(2.40, width / max(height, 1e-6)))
    if ratio >= 1.0:
        return (768, max(320, int(round(768 / ratio))))
    return (max(384, int(round(768 * ratio))), 768)


def _write_category_texture(path: Path, spec: AssetSpec) -> None:
    """Create item-specific art using a category-appropriate packaging system."""
    if Image is None:
        raise RuntimeError(
            f"Pillow=={REQUIRED_PILLOW_VERSION} is required; refusing to write fallback texture {path}"
        )
    width, height = _new_artwork_size(spec)
    system = _new_art_system(spec)
    base = tuple(int(round(value * 255)) for value in spec.color)
    accent = tuple(int(round(value * 255)) for value in spec.accent)
    ink = _blend(base, (8, 14, 18), 0.72)
    paper = _blend(base, (255, 252, 239), 0.62)
    seed = hashlib.sha256(spec.asset_key.encode("utf-8")).digest()
    image = Image.new("RGB", (width, height), paper)
    draw = ImageDraw.Draw(image)

    # Category systems use different background print processes.  The seed
    # varies rhythm within a system without turning label color into identity.
    if system == "carton":
        stripe = max(8, width // (11 + seed[0] % 7))
        for index, x in enumerate(range(-height, width + height, stripe * 3)):
            tone = _blend(base, accent, 0.22 + 0.10 * ((index + seed[1]) % 3))
            draw.polygon(((x, 0), (x + stripe, 0), (x + height + stripe, height), (x + height, height)), fill=tone)
    elif system == "chilled":
        for index in range(7):
            y = index * height // 7
            draw.rectangle((0, y, width, y + max(5, height // 28)), fill=_blend(base, accent, .12 + .05 * (index % 3)))
    elif system == "bakery":
        image.paste(_blend(base, (245, 224, 182), .55), (0, 0, width, height))
        for index in range(9):
            x = (index * width // 8 + seed[0]) % width
            draw.line((x, 0, x - width * .18, height), fill=_blend(base, ink, .22), width=max(4, width // 80))
    elif system == "bottle":
        for y in range(height):
            draw.line((0, y, width, y), fill=_blend(paper, base, .10 + .30 * y / height))
    elif system == "wrap":
        for index, x in enumerate(range(0, width, max(18, width // 14))):
            draw.rectangle((x, 0, x + max(6, width // 42), height), fill=_blend(base, accent, .18 + .08 * (index % 2)))
    elif system == "soft_pack":
        image.paste(_blend(base, (232, 214, 174), .62), (0, 0, width, height))
        for index in range(70):
            x = (seed[index % len(seed)] * (index + 7)) % width
            y = (seed[(index + 9) % len(seed)] * (index + 13)) % height
            draw.ellipse((x, y, x + 3, y + 3), fill=_blend(ink, base, .55))
    elif system == "household":
        image.paste(_blend(base, (238, 247, 246), .48), (0, 0, width, height))
    elif system == "produce":
        tile = max(24, min(width, height) // 8)
        for y in range(0, height, tile):
            for x in range(0, width, tile):
                if (x // tile + y // tile + seed[0]) % 2:
                    draw.rectangle((x, y, x + tile, y + tile), fill=_blend(base, accent, .16))
    else:
        image.paste((188, 190, 183), (0, 0, width, height))

    margin = max(18, int(min(width, height) * 0.055))
    title_size = max(28, int(min(width, height) * (0.105 if len(spec.product_name) < 15 else 0.078)))
    small_size = max(16, int(min(width, height) * 0.039))
    taglines = {
        "fixture": ("AISLE HARDWARE", "MERCHANDISING SYSTEM", "RETAIL FIXTURE"),
        "produce": ("FARM LOT", "HARVESTED TODAY", "MARKET SELECT"),
        "household": ("HOME CARE", "CLEAN ROUTINE", "UTILITY SERIES"),
        "chilled": ("KEEP CHILLED", "CREAMERY BATCH", "FRESH DAIRY"),
        "bakery": ("BAKED & SEALED", "DAILY BAKE", "BAKERY LOAF"),
        "bottle": ("BOTTLED IN SMALL RUNS", "SERVE COLD", "PANTRY POUR"),
        "wrap": ("PANTRY LOT", "SEALED FRESH", "SMALL BATCH"),
        "soft_pack": ("FRESH PACK", "PANTRY STAPLE", "SEALED AT SOURCE"),
        "carton": ("CUPBOARD SERIES", "FAMILY SIZE", "EVERYDAY TABLE"),
    }
    tagline = taglines[system][seed[2] % 3]
    if system == "bottle" and spec.category == "beverage":
        tagline = ("ELECTROLYTE BLEND", "SERVE COLD", "ACTIVE REFRESH")[seed[2] % 3]
    brand, tagline, layout = ITEM_ART_DIRECTIONS[spec.asset_key]

    if system == "fixture":
        draw.rounded_rectangle((margin, margin, width - margin, height - margin), radius=18, fill=(28, 34, 39), outline=accent, width=5)
        for x in range(margin * 2, width - margin * 2, max(28, width // 16)):
            draw.line((x, height * .22, x, height * .78), fill=(72, 82, 88), width=2)
        if spec.asset_key == "price_display":
            draw.text((width * .58, height * .28), "$3.49", font=_font(int(height * .34), True), fill=(236, 244, 226))
    elif system == "produce":
        draw.rectangle((margin, margin, width - margin, height - margin), outline=ink, width=5)
        for index in range(5 + seed[3] % 4):
            radius = int(min(width, height) * (0.045 + 0.008 * (index % 3)))
            cx = margin * 2 + (index * (width - margin * 4) // max(1, 4 + seed[3] % 3))
            cy = int(height * (0.62 + 0.09 * ((index + seed[4]) % 2)))
            draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), fill=accent, outline=ink, width=3)
            draw.line((cx, cy - radius, cx + radius // 3, cy - radius - radius // 2), fill=ink, width=4)
    elif system == "household":
        draw.polygon(((0, int(height * .58)), (width, int(height * .34)), (width, height), (0, height)), fill=accent)
        for index in range(4):
            radius = int(min(width, height) * (0.055 + index * .014))
            cx = int(width * (.70 + .06 * ((index + seed[5]) % 2)))
            cy = int(height * (.18 + index * .11))
            draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), outline=(250, 250, 245), width=5)
        draw.rounded_rectangle((margin, int(height * .74), width - margin, int(height * .90)), radius=12, fill=(244, 247, 239))
        draw.text((margin * 1.4, int(height * .77)), "DIRECTIONS  •  TESTED FOR HOME USE", font=_font(small_size, True), fill=ink)
    elif system == "chilled":
        draw.rectangle((0, 0, width, int(height * .18)), fill=accent)
        draw.ellipse((int(width * .57), int(height * .42), int(width * .94), int(height * .79)), fill=_blend(accent, (255, 255, 255), .28), outline=ink, width=5)
        for offset in range(3):
            y = int(height * (.68 + offset * .055))
            draw.arc((margin, y - height * .18, width * .55, y + height * .10), 195, 342, fill=ink, width=5)
    elif system == "bakery":
        draw.rectangle((0, 0, width, int(height * .18)), fill=accent)
        draw.rounded_rectangle(
            (int(width * .63), int(height * .52), int(width * .92), int(height * .76)),
            radius=max(8, min(width, height) // 24),
            fill=_blend(accent, (255, 246, 218), .42), outline=ink, width=5,
        )
        for offset in range(3):
            x = int(width * (.69 + offset * .075))
            draw.line((x, int(height * .55), x - int(width * .06), int(height * .69)), fill=ink, width=4)
    elif system == "bottle":
        draw.rectangle((0, 0, int(width * BOTTLE_LEFT_STRIP_FRACTION), height), fill=ink)
        draw.rectangle((int(width * .84), 0, width, height), fill=accent)
        for offset in range(4):
            y = int(height * (.45 + offset * .075))
            draw.arc((int(width * .25), y - int(height * .16), int(width * .82), y + int(height * .12)), 195, 345, fill=accent, width=6)
        draw.ellipse((int(width * .62), int(height * .15), int(width * .87), int(height * .36)), outline=ink, width=5)
    elif system == "wrap":
        draw.rectangle((0, int(height * .08), width, int(height * .22)), fill=ink)
        draw.rectangle((0, int(height * .78), width, int(height * .94)), fill=accent)
        for index in range(7):
            x = int(width * (.08 + index * .14))
            draw.line((x, int(height * .30), x + int(width * .08), int(height * .68)), fill=_blend(ink, accent, .45), width=4)
    elif system == "soft_pack":
        draw.rectangle((margin, margin, width - margin, height - margin), outline=ink, width=4)
        draw.line((margin, int(height * .15), width - margin, int(height * .15)), fill=accent, width=10)
        draw.line((margin, int(height * .87), width - margin, int(height * .87)), fill=accent, width=10)
        for index in range(5):
            x = int(width * (.18 + index * .15))
            draw.ellipse((x - 12, int(height * .62), x + 12, int(height * .72)), fill=ink)
    else:
        draw.rectangle((0, 0, width, int(height * .17)), fill=ink)
        draw.rectangle((0, int(height * .82), width, height), fill=accent)
        badge_radius = int(min(width, height) * .13)
        badge_x, badge_y = int(width * .76), int(height * .46)
        draw.regular_polygon((badge_x, badge_y, badge_radius), 6, rotation=30, fill=accent, outline=ink)

    # Six explicit compositions alter hierarchy, alignment, and panel shape;
    # the item direction table selects them deliberately rather than by color.
    if layout == 0:
        text_x, title_y, anchor = margin, int(height * .30), "la"
        draw.rectangle((0, 0, width, int(height * .12)), fill=accent)
    elif layout == 1:
        text_x, title_y, anchor = width // 2, int(height * .34), "ma"
        draw.rounded_rectangle((margin, int(height * .24), width - margin, int(height * .62)), radius=18, fill=_blend(paper, (255, 255, 255), .35), outline=ink, width=4)
    elif layout == 2:
        text_x, title_y, anchor = int(width * .24), int(height * .24), "la"
        draw.polygon(((0, 0), (int(width * .42), 0), (int(width * .26), height), (0, height)), fill=_blend(accent, ink, .18))
    elif layout == 3:
        text_x, title_y, anchor = width // 2, int(height * .40), "ma"
        draw.ellipse((int(width * .12), int(height * .22), int(width * .88), int(height * .72)), fill=_blend(paper, (255, 255, 255), .45), outline=accent, width=7)
    elif layout == 4:
        text_x, title_y, anchor = width - margin, int(height * .27), "ra"
        draw.rectangle((0, int(height * .70), width, height), fill=_blend(accent, ink, .10))
    else:
        text_x = (
            int(width * BOTTLE_LAYOUT_FIVE_TEXT_INSET_FRACTION)
            if system == "bottle" else margin
        )
        title_y, anchor = int(height * .38), "la"
        draw.polygon(((0, 0), (width, 0), (width, int(height * .24)), (int(width * .35), int(height * .34)), (0, int(height * .24))), fill=ink)

    if system == "fixture":
        text_x, anchor = margin, "la"
        title_y = int(height * .18)
    brand_y = margin if layout != 5 else int(height * .07)
    brand_fill = (247, 246, 235) if layout == 5 or system == "fixture" else ink
    if system == "wrap" and layout == 1:
        # The wrap system's upper ink bar is a deliberate brand strip.  Keep
        # centered layout-one branding inside it instead of obscuring it with
        # dark-on-dark text at the outer margin.
        brand_y = int(height * .12)
        brand_fill = (247, 246, 235)
    draw.text((text_x, brand_y), brand, font=_font(small_size, True), fill=brand_fill, anchor=anchor)
    draw.multiline_text(
        (text_x, title_y), spec.product_name.replace(" ", "\n", 1),
        font=_font(title_size, True), fill=(246, 247, 239) if system == "fixture" else ink,
        spacing=2, anchor=anchor,
    )
    tagline_y = int(height * (.78 if layout == 4 else .68))
    draw.text((text_x, tagline_y), tagline, font=_font(small_size, True), fill=(246, 247, 239) if system == "fixture" else ink, anchor=anchor)
    if system not in {"produce", "fixture"}:
        # Item-specific lot bars give each wrap/label a different side rhythm.
        bar_y = int(height * .91)
        for index in range(18):
            bar_width = 1 + seed[index % len(seed)] % 4
            x = margin + index * max(4, (width - 2 * margin) // 22)
            draw.rectangle((x, bar_y, x + bar_width, min(height - margin // 2, bar_y + int(height * .055))), fill=ink)
    draw.text((width - margin, height - margin // 2), spec.asset_key.upper(), font=_font(max(12, small_size // 2)), fill=ink, anchor="rs")
    image.save(path, format="PNG", optimize=True)


def _write_surface_maps(normal_path: Path, roughness_path: Path, spec: AssetSpec, size: int = 128) -> None:
    """Author category-scale tangent normals and roughness, not scalar-only PBR."""
    if Image is None:
        raise RuntimeError(f"Pillow=={REQUIRED_PILLOW_VERSION} is required for surface maps")
    system = _new_art_system(spec)
    seed = hashlib.sha256((spec.asset_key + ":surface").encode("utf-8")).digest()
    normal = Image.new("RGB", (size, size), (128, 128, 252))
    roughness_base = {
        "produce": 205, "fixture": 92, "household": 112, "chilled": 138,
        "bakery": 188, "bottle": 82, "wrap": 124, "soft_pack": 176, "carton": 164,
    }[system]
    roughness = Image.new("L", (size, size), roughness_base)
    normal_draw = ImageDraw.Draw(normal)
    rough_draw = ImageDraw.Draw(roughness)

    if system in {"carton", "soft_pack", "bakery", "chilled"}:
        for index in range(28):
            y = (seed[index % len(seed)] + index * 17) % size
            tilt = (seed[(index + 7) % len(seed)] % 9) - 4
            normal_draw.line((0, y, size, y + tilt), fill=(112 + index % 12, 138 - index % 9, 248), width=1)
            rough_draw.line((0, y, size, y + tilt), fill=min(240, roughness_base + 22), width=1)
    elif system in {"wrap", "fixture"}:
        for x in range(0, size, 4 + seed[0] % 3):
            normal_draw.line((x, 0, x, size), fill=(118, 132, 250), width=1)
            rough_draw.line((x, 0, x, size), fill=max(55, roughness_base - 25), width=1)
    elif system in {"bottle", "household"}:
        for index in range(96):
            x = (seed[index % len(seed)] * (index + 3)) % size
            y = (seed[(index + 11) % len(seed)] * (index + 5)) % size
            radius = 1 + seed[(index + 19) % len(seed)] % 2
            normal_draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(136, 120, 249))
            rough_draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=min(220, roughness_base + 30))
    else:
        for index in range(140):
            x = (seed[index % len(seed)] * (index + 7)) % size
            y = (seed[(index + 13) % len(seed)] * (index + 9)) % size
            normal_draw.point((x, y), fill=(116 + seed[index % len(seed)] % 25, 116 + seed[(index + 1) % len(seed)] % 25, 246))
            rough_draw.point((x, y), fill=max(120, min(245, roughness_base + (seed[index % len(seed)] % 31) - 15)))

    normal.save(normal_path, format="PNG", optimize=True)
    roughness.save(roughness_path, format="PNG", optimize=True)


def _material(
    name: str,
    color: tuple[float, float, float],
    texture_path: str | None = None,
    roughness: float = 0.58,
    metallic: float = 0.0,
    opacity: float = 1.0,
    normal_texture_path: str | None = None,
    roughness_texture_path: str | None = None,
) -> str:
    safe = name.replace("-", "_")
    lines = [
        f'        def Material "{safe}" {{',
        f'            token outputs:surface.connect = </Asset/Looks/{safe}/Preview.outputs:surface>',
    ]
    if texture_path:
        lines += [
            '            def Shader "StReader" {',
            '                uniform token info:id = "UsdPrimvarReader_float2"',
            '                token inputs:varname = "st"',
            '                float2 outputs:result',
            '            }',
            '            def Shader "Texture" {',
            '                uniform token info:id = "UsdUVTexture"',
            f'                asset inputs:file = @{texture_path}@',
            '                float2 inputs:st.connect = </Asset/Looks/' + safe + '/StReader.outputs:result>',
            '                float3 outputs:rgb',
            '            }',
        ]
    if normal_texture_path:
        lines += [
            '            def Shader "NormalTexture" {',
            '                uniform token info:id = "UsdUVTexture"',
            f'                asset inputs:file = @{normal_texture_path}@',
            '                token inputs:sourceColorSpace = "raw"',
            '                float4 inputs:scale = (2, 2, 2, 1)',
            '                float4 inputs:bias = (-1, -1, -1, 0)',
            '                float2 inputs:st.connect = </Asset/Looks/' + safe + '/StReader.outputs:result>',
            '                float3 outputs:rgb',
            '            }',
        ]
    if roughness_texture_path:
        lines += [
            '            def Shader "RoughnessTexture" {',
            '                uniform token info:id = "UsdUVTexture"',
            f'                asset inputs:file = @{roughness_texture_path}@',
            '                token inputs:sourceColorSpace = "raw"',
            '                float2 inputs:st.connect = </Asset/Looks/' + safe + '/StReader.outputs:result>',
            '                float outputs:r',
            '            }',
        ]
    lines += [
        '            def Shader "Preview" {',
        '                uniform token info:id = "UsdPreviewSurface"',
        f'                color3f inputs:diffuseColor = ({color[0]:.5f}, {color[1]:.5f}, {color[2]:.5f})',
        f'                float inputs:roughness = {roughness:.4f}',
        f'                float inputs:metallic = {metallic:.4f}',
    ]
    if opacity < 1.0:
        lines.append(f'                float inputs:opacity = {opacity:.4f}')
    if texture_path:
        lines.append(f'                color3f inputs:diffuseColor.connect = </Asset/Looks/{safe}/Texture.outputs:rgb>')
    if normal_texture_path:
        lines.append(f'                normal3f inputs:normal.connect = </Asset/Looks/{safe}/NormalTexture.outputs:rgb>')
    if roughness_texture_path:
        lines.append(f'                float inputs:roughness.connect = </Asset/Looks/{safe}/RoughnessTexture.outputs:r>')
    lines += ['                token outputs:surface', '            }', '        }']
    return "\n".join(lines)


def _front_panel(
    width: float,
    depth: float,
    height: float,
    uv_bounds: tuple[float, float, float, float] | None = None,
) -> str:
    y = depth / 2.0 + 0.002
    w = width * 0.88 / 2.0
    h = height * 0.84 / 2.0
    uv = (
        "[(0, 0), (1, 0), (1, 1), (0, 1)]"
        if uv_bounds is None else
        "[(%.6f, %.6f), (%.6f, %.6f), (%.6f, %.6f), (%.6f, %.6f)]" % (
            uv_bounds[0], uv_bounds[1], uv_bounds[2], uv_bounds[1],
            uv_bounds[2], uv_bounds[3], uv_bounds[0], uv_bounds[3],
        )
    )
    return f'''        def Mesh "FrontPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [(-{w:.5f}, {y:.5f}, -{h:.5f}), ({w:.5f}, {y:.5f}, -{h:.5f}), ({w:.5f}, {y:.5f}, {h:.5f}), (-{w:.5f}, {y:.5f}, {h:.5f})]
            int[] faceVertexCounts = [4]
            # Winding and the authored +Y normal must agree; viewed from the
            # local front (+Y), this order faces outward and preserves upright UVs.
            int[] faceVertexIndices = [0, 3, 2, 1]
            normal3f[] normals = [(0, 1, 0)] (
                interpolation = "uniform"
            )
            texCoord2f[] primvars:st = {uv} (
                interpolation = "vertex"
            )
            rel material:binding = </Asset/Looks/Front>
        }}'''


def _cube(name: str, size: tuple[float, float, float], translate: tuple[float, float, float], material: str) -> str:
    # Isaac's USD bounds/render path does not apply nonuniform xform ops
    # authored directly on a Gprim. Put the scale on an Xform parent so the
    # physical dimensions remain correct in both USD consumers.
    return f'''        def Xform "{name}" {{
            double3 xformOp:translate = ({translate[0]:.5f}, {translate[1]:.5f}, {translate[2]:.5f})
            double3 xformOp:scale = ({size[0] / 2.0:.5f}, {size[1] / 2.0:.5f}, {size[2] / 2.0:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            def Cube "Shape" (
                prepend apiSchemas = ["MaterialBindingAPI"]
            ) {{
                double size = 2.0
                rel material:binding = </Asset/Looks/{material}>
            }}
        }}'''


def _beveled_box(name: str, size: tuple[float, float, float], bevel: float, material: str) -> str:
    """Create a chamfered rectangular shell with stable, explicit dimensions."""
    width, depth, height = size
    half_w, half_d, half_h = width / 2.0, depth / 2.0, height / 2.0
    b = min(bevel, width * 0.18, depth * 0.18)
    ring = [
        (-half_w + b, -half_d), (half_w - b, -half_d),
        (half_w, -half_d + b), (half_w, half_d - b),
        (half_w - b, half_d), (-half_w + b, half_d),
        (-half_w, half_d - b), (-half_w, -half_d + b),
    ]
    points = [(x, y, -half_h) for x, y in ring] + [(x, y, half_h) for x, y in ring]
    faces = [[index for index in reversed(range(8))], list(range(8, 16))]
    for index in range(8):
        next_index = (index + 1) % 8
        faces.append([index, next_index, next_index + 8, index + 8])
    counts = ", ".join("8" if index < 2 else "4" for index in range(len(faces)))
    indices = ", ".join(str(vertex) for face in faces for vertex in face)
    point_text = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in points)
    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [{point_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _cylinder(name: str, radius: float, height: float, material: str, translate_z: float = 0.0, vertices: int = 32) -> str:
    translate = f'''\n            double3 xformOp:translate = (0, 0, {translate_z:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate"]''' if abs(translate_z) > 1e-9 else ""
    return f'''        def Cylinder "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            int vertices = {vertices}
            double radius = {radius:.5f}
            double height = {height:.5f}{translate}
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _side_and_back_print_panels(width: float, depth: float, height: float) -> list[str]:
    """UV-authored side nutrition panels and a printed back information panel."""
    half_h = height * 0.27
    half_side_depth = depth * 0.36
    x_offset = width / 2.0 + 0.001
    y_offset = depth / 2.0 + 0.001
    side_uv = "[(0.05, 0.12), (0.34, 0.12), (0.34, 0.88), (0.05, 0.88)]"
    back_uv = "[(0.08, 0.08), (0.92, 0.08), (0.92, 0.34), (0.08, 0.34)]"
    return [
        f'''        def Mesh "LeftPrintPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [(-{x_offset:.5f}, -{half_side_depth:.5f}, -{half_h:.5f}), (-{x_offset:.5f}, {half_side_depth:.5f}, -{half_h:.5f}), (-{x_offset:.5f}, {half_side_depth:.5f}, {half_h:.5f}), (-{x_offset:.5f}, -{half_side_depth:.5f}, {half_h:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            normal3f[] normals = [(-1, 0, 0)] (interpolation = "uniform")
            texCoord2f[] primvars:st = {side_uv} (interpolation = "vertex")
            rel material:binding = </Asset/Looks/Print>
        }}''',
        f'''        def Mesh "RightPrintPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [({x_offset:.5f}, {half_side_depth:.5f}, -{half_h:.5f}), ({x_offset:.5f}, -{half_side_depth:.5f}, -{half_h:.5f}), ({x_offset:.5f}, -{half_side_depth:.5f}, {half_h:.5f}), ({x_offset:.5f}, {half_side_depth:.5f}, {half_h:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            normal3f[] normals = [(1, 0, 0)] (interpolation = "uniform")
            texCoord2f[] primvars:st = {side_uv} (interpolation = "vertex")
            rel material:binding = </Asset/Looks/Print>
        }}''',
        f'''        def Mesh "BackPrintPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [(-{width * .38:.5f}, -{y_offset:.5f}, -{height * .38:.5f}), ({width * .38:.5f}, -{y_offset:.5f}, -{height * .38:.5f}), ({width * .38:.5f}, -{y_offset:.5f}, -{height * .22:.5f}), (-{width * .38:.5f}, -{y_offset:.5f}, -{height * .22:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 3, 2, 1]
            normal3f[] normals = [(0, -1, 0)] (interpolation = "uniform")
            texCoord2f[] primvars:st = {back_uv} (interpolation = "vertex")
            rel material:binding = </Asset/Looks/Print>
        }}''',
    ]


def _hero_side_and_back_print_panels(spec: AssetSpec, material: str) -> list[str]:
    """Add near-full side/back print faces mapped to the hero wrap atlas."""
    width, depth, height = spec.dimensions_m
    regions = _hero_wrap_regions(spec)
    half_h = height * .42
    half_side_depth = depth * .44
    x_offset = width / 2.0 + .001
    y_offset = depth / 2.0 + .001

    def uv(bounds: tuple[float, float, float, float], reverse_u: bool = False) -> str:
        low_u, high_u = bounds[0], bounds[2]
        if reverse_u:
            low_u, high_u = high_u, low_u
        return "[(%.6f, %.6f), (%.6f, %.6f), (%.6f, %.6f), (%.6f, %.6f)]" % (
            low_u, bounds[1], high_u, bounds[1], high_u, bounds[3], low_u, bounds[3],
        )

    return [
        f'''        def Mesh "HeroLeftPrintPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [(-{x_offset:.5f}, -{half_side_depth:.5f}, -{half_h:.5f}), (-{x_offset:.5f}, {half_side_depth:.5f}, -{half_h:.5f}), (-{x_offset:.5f}, {half_side_depth:.5f}, {half_h:.5f}), (-{x_offset:.5f}, -{half_side_depth:.5f}, {half_h:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 3, 2, 1]
            normal3f[] normals = [(-1, 0, 0)] (interpolation = "uniform")
            texCoord2f[] primvars:st = {uv(regions["left"])} (interpolation = "vertex")
            rel material:binding = </Asset/Looks/{material}>
        }}''',
        f'''        def Mesh "HeroRightPrintPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [({x_offset:.5f}, {half_side_depth:.5f}, -{half_h:.5f}), ({x_offset:.5f}, -{half_side_depth:.5f}, -{half_h:.5f}), ({x_offset:.5f}, -{half_side_depth:.5f}, {half_h:.5f}), ({x_offset:.5f}, {half_side_depth:.5f}, {half_h:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 3, 2, 1]
            normal3f[] normals = [(1, 0, 0)] (interpolation = "uniform")
            texCoord2f[] primvars:st = {uv(regions["right"])} (interpolation = "vertex")
            rel material:binding = </Asset/Looks/{material}>
        }}''',
        f'''        def Mesh "HeroBackPrintPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [(-{width * .44:.5f}, -{y_offset:.5f}, -{half_h:.5f}), ({width * .44:.5f}, -{y_offset:.5f}, -{half_h:.5f}), ({width * .44:.5f}, -{y_offset:.5f}, {half_h:.5f}), (-{width * .44:.5f}, -{y_offset:.5f}, {half_h:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            normal3f[] normals = [(0, -1, 0)] (interpolation = "uniform")
            texCoord2f[] primvars:st = {uv(regions["back"], reverse_u=True)} (interpolation = "vertex")
            rel material:binding = </Asset/Looks/{material}>
        }}''',
    ]


def _cylinder_at(
    name: str,
    radius: float,
    height: float,
    material: str,
    translate: tuple[float, float, float],
    axis: str = "Z",
    vertices: int = 32,
) -> str:
    return f'''        def Cylinder "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            uniform token axis = "{axis}"
            int vertices = {vertices}
            double radius = {radius:.5f}
            double height = {height:.5f}
            double3 xformOp:translate = ({translate[0]:.5f}, {translate[1]:.5f}, {translate[2]:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _ellipsoid(
    name: str,
    size: tuple[float, float, float],
    translate: tuple[float, float, float],
    material: str,
) -> str:
    return f'''        def Xform "{name}" {{
            double3 xformOp:translate = ({translate[0]:.5f}, {translate[1]:.5f}, {translate[2]:.5f})
            double3 xformOp:scale = ({size[0]:.5f}, {size[1]:.5f}, {size[2]:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            def Sphere "Shape" (
                prepend apiSchemas = ["MaterialBindingAPI"]
            ) {{
                double radius = 0.5
                rel material:binding = </Asset/Looks/{material}>
            }}
        }}'''


def _oval_prism(
    name: str,
    size: tuple[float, float, float],
    material: str,
    translate_z: float = 0.0,
    vertices: int = 32,
) -> str:
    """Closed oval tub/cup mesh with straight walls and explicit bounds."""
    width, depth, height = size
    lower = [
        (width / 2.0 * math.cos(2.0 * math.pi * index / vertices),
         depth / 2.0 * math.sin(2.0 * math.pi * index / vertices),
         translate_z - height / 2.0)
        for index in range(vertices)
    ]
    upper = [(x, y, translate_z + height / 2.0) for x, y, _ in lower]
    points = lower + upper
    faces = [list(reversed(range(vertices))), list(range(vertices, 2 * vertices))]
    faces.extend(
        [index, (index + 1) % vertices, (index + 1) % vertices + vertices, index + vertices]
        for index in range(vertices)
    )
    counts = ", ".join(str(len(face)) for face in faces)
    indices = ", ".join(str(vertex) for face in faces for vertex in face)
    point_text = ", ".join(f"({x:.6f}, {y:.6f}, {z:.6f})" for x, y, z in points)
    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [{point_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _curved_banana_finger(name: str, size: tuple[float, float, float], finger: int, material: str) -> str:
    """One continuous tapered tube along a curved centerline."""
    width, depth, height = size
    rings, sides = 9, 10
    points: list[tuple[float, float, float]] = []
    for ring in range(rings):
        t = ring / (rings - 1)
        center_x = (finger - 2) * width * 0.15 + math.sin(math.pi * t) * width * (0.055 + finger * 0.004)
        center_y = (finger - 2) * depth * 0.055
        center_z = -height / 2.0 + t * height * 0.78
        radius_x = width * (0.052 - 0.015 * t)
        radius_y = depth * (0.105 - 0.025 * t)
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            points.append((center_x + radius_x * math.cos(angle), center_y + radius_y * math.sin(angle), center_z))
    faces: list[list[int]] = [list(reversed(range(sides))), list(range((rings - 1) * sides, rings * sides))]
    for ring in range(rings - 1):
        for side in range(sides):
            nxt = (side + 1) % sides
            base = ring * sides
            upper = (ring + 1) * sides
            faces.append([base + side, base + nxt, upper + nxt, upper + side])
    return _mesh_from_faces(name, points, faces, material)


def _irregular_floret(name: str, size: tuple[float, float, float], translate: tuple[float, float, float], material: str) -> str:
    """Lobed closed floret mesh with deterministic irregular rings."""
    width, depth, height = size
    rings, sides = 4, 12
    points: list[tuple[float, float, float]] = []
    for ring in range(rings):
        t = ring / (rings - 1)
        z = translate[2] - height / 2.0 + t * height
        envelope = math.sin(math.pi * (0.12 + 0.76 * t))
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            lobe = 0.82 + 0.16 * math.sin(5.0 * angle + ring * 0.9)
            points.append((
                translate[0] + width / 2.0 * envelope * lobe * math.cos(angle),
                translate[1] + depth / 2.0 * envelope * lobe * math.sin(angle),
                z,
            ))
    faces: list[list[int]] = [list(reversed(range(sides))), list(range((rings - 1) * sides, rings * sides))]
    for ring in range(rings - 1):
        for side in range(sides):
            nxt = (side + 1) % sides
            faces.append([ring * sides + side, ring * sides + nxt, (ring + 1) * sides + nxt, (ring + 1) * sides + side])
    return _mesh_from_faces(name, points, faces, material)


def _loaf_mesh(name: str, size: tuple[float, float, float], material: str) -> str:
    """Continuous scored loaf envelope with a flat base and domed crown."""
    width, depth, height = size
    rings, sides = 11, 14
    points: list[tuple[float, float, float]] = []
    for ring in range(rings):
        t = ring / (rings - 1)
        x = -width / 2.0 + t * width
        end_taper = max(0.18, math.sin(math.pi * t) ** 0.42)
        score = 1.0 - 0.045 * sum(math.exp(-((t - mark) / 0.035) ** 2) for mark in (0.32, 0.50, 0.68))
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            y = depth / 2.0 * end_taper * math.cos(angle)
            z = -height / 2.0 + height * 0.50 * (1.0 + math.sin(angle)) * end_taper * score
            points.append((x, y, z))
    faces: list[list[int]] = [list(reversed(range(sides))), list(range((rings - 1) * sides, rings * sides))]
    for ring in range(rings - 1):
        for side in range(sides):
            nxt = (side + 1) % sides
            faces.append([ring * sides + side, ring * sides + nxt, (ring + 1) * sides + nxt, (ring + 1) * sides + side])
    return _mesh_from_faces(name, points, faces, material)


def _mesh_from_faces(
    name: str,
    points: list[tuple[float, float, float]],
    faces: list[list[int]],
    material: str,
) -> str:
    point_text = ", ".join(f"({x:.6f}, {y:.6f}, {z:.6f})" for x, y, z in points)
    counts = ", ".join(str(len(face)) for face in faces)
    indices = ", ".join(str(vertex) for face in faces for vertex in face)
    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [{point_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _woven_strip(name: str, width: float, depth: float, height: float, index: int, material: str) -> str:
    """Extruded diagonal wicker ribbon with alternating over/under depth."""
    face_sign = -1.0 if name.startswith("Back") else 1.0
    center_y = face_sign * (depth / 2.0 - (0.004 if index % 2 else 0.010))
    half_depth = 0.003
    half_strip = 0.008
    start_x = -width / 2.0 + 0.02 + index * (width - 0.04) / 7.0
    slope = height * (0.42 if index % 2 == 0 else -0.42)
    x0 = max(-width / 2.0 + 0.008, start_x - width * 0.18)
    x1 = min(width / 2.0 - 0.008, start_x + width * 0.18)
    z0 = max(-height / 2.0 + 0.02, -slope / 2.0)
    z1 = min(height / 2.0 - 0.02, slope / 2.0)
    front = ((x0, center_y + half_depth, z0 - half_strip), (x1, center_y + half_depth, z1 - half_strip),
             (x1, center_y + half_depth, z1 + half_strip), (x0, center_y + half_depth, z0 + half_strip))
    back = tuple((x, center_y - half_depth, z) for x, _, z in front)
    points = list(front + back)
    faces = [[0, 1, 2, 3], [7, 6, 5, 4], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
    return _mesh_from_faces(name, points, faces, material)


def _wedge(
    name: str,
    size: tuple[float, float, float],
    material: str,
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> str:
    """Triangular prism with its full declared width/depth/height bounds."""
    width, depth, height = size
    x0, x1 = -width / 2.0 + translate[0], width / 2.0 + translate[0]
    y0, y1 = -depth / 2.0 + translate[1], depth / 2.0 + translate[1]
    z0, z1 = -height / 2.0 + translate[2], height / 2.0 + translate[2]
    points = ((x0, y0, z0), (x1, y0, z0), (x0, y0, z1),
              (x0, y1, z0), (x1, y1, z0), (x0, y1, z1))
    faces = ((0, 1, 2), (3, 5, 4), (0, 3, 4, 1), (1, 4, 5, 2), (2, 5, 3, 0))
    point_text = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in points)
    counts = ", ".join(str(len(face)) for face in faces)
    indices = ", ".join(str(index) for face in faces for index in face)
    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [{point_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _pouch(name: str, size: tuple[float, float, float], material: str, seam_material: str = "Front") -> list[str]:
    """Soft package silhouette with a rounded belly, gusset, and sealed seams."""
    width, depth, height = size
    return [
        _ellipsoid(name + "Belly", (width, depth, height * 0.88), (0.0, 0.0, -height * 0.01), material),
        _cube(name + "TopSeal", (width * 0.88, depth * 0.35, height * 0.055), (0.0, 0.0, height * 0.4625), seam_material),
        _cube(name + "BottomGusset", (width * 0.82, depth * 0.72, height * 0.06), (0.0, 0.0, -height * 0.47), seam_material),
        _cube(name + "LeftSeam", (width * 0.035, depth * 0.30, height * 0.72), (-width * 0.465, 0.0, 0.0), seam_material),
        _cube(name + "RightSeam", (width * 0.035, depth * 0.30, height * 0.72), (width * 0.465, 0.0, 0.0), seam_material),
    ]


def _frustum(
    name: str,
    radius_bottom: float,
    radius_top: float,
    height: float,
    material: str,
    translate_z: float,
    vertices: int = 40,
    translate_x: float = 0.0,
    translate_y: float = 0.0,
) -> str:
    """Author a closed, tapered shoulder mesh without invalid cone fields."""
    bottom = [
        (translate_x + radius_bottom * math.cos(2.0 * math.pi * index / vertices),
         translate_y + radius_bottom * math.sin(2.0 * math.pi * index / vertices),
         translate_z - height / 2.0)
        for index in range(vertices)
    ]
    top = [
        (translate_x + radius_top * math.cos(2.0 * math.pi * index / vertices),
         translate_y + radius_top * math.sin(2.0 * math.pi * index / vertices),
         translate_z + height / 2.0)
        for index in range(vertices)
    ]
    points = bottom + top
    faces = [list(reversed(range(vertices))), list(range(vertices, 2 * vertices))]
    faces.extend(
        [index, (index + 1) % vertices, (index + 1) % vertices + vertices, index + vertices]
        for index in range(vertices)
    )
    counts = ", ".join(str(len(face)) for face in faces)
    indices = ", ".join(str(vertex) for face in faces for vertex in face)
    point_text = ", ".join(f"({x:.6f}, {y:.6f}, {z:.6f})" for x, y, z in points)
    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [{point_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _asset_usda(
    spec: AssetSpec,
    texture_name: str,
    normal_texture_name: str | None = None,
    roughness_texture_name: str | None = None,
) -> str:
    width, depth, height = spec.dimensions_m
    body_material = "Body"
    front_material = "Front"
    texture_path = f"../textures/{texture_name}"
    normal_texture_path = f"../textures/{normal_texture_name}" if normal_texture_name else None
    roughness_texture_path = f"../textures/{roughness_texture_name}" if roughness_texture_name else None
    is_hero_asset = spec.asset_key in HERO_ART_DIRECTIONS
    hero_front_uv = _hero_wrap_regions(spec)["front"] if is_hero_asset else None

    def asset_front_panel(panel_width: float, panel_depth: float, panel_height: float) -> str:
        return _front_panel(panel_width, panel_depth, panel_height, hero_front_uv)

    geometry: list[str] = []
    if spec.model_type in {"box", "carton", "sign"}:
        geometry.append(_beveled_box("BodyShell", spec.dimensions_m, min(width, depth) * 0.075, body_material))
        geometry.append(asset_front_panel(width, depth, height))
        geometry.append(_cube("TopBand", (width * 0.86, 0.012, height * 0.10), (0.0, depth / 2.0 + 0.008, height * 0.34), front_material))
        geometry.append(_cube("BottomTrim", (width * 0.82, 0.010, height * 0.025), (0.0, depth / 2.0 + 0.009, -height * 0.40), "Metal"))
        if spec.model_type == "carton":
            geometry.append(_beveled_box("CartonTop", (width * 0.84, depth * 0.88, height * 0.075), min(width, depth) * 0.05, front_material))
            geometry.append(_cylinder("PourCap", width * 0.15, height * 0.055, "Metal", height * 0.4725, vertices=24))
    elif spec.model_type in {"can", "jar"}:
        radius = min(width, depth) / 2.0
        geometry.append(_cylinder("Body", radius, height, body_material, vertices=40))
        if spec.model_type == "jar":
            geometry.append(_cylinder("Shoulder", radius * 0.92, height * 0.08, body_material, height * 0.39, vertices=40))
            geometry.append(_cylinder("Lid", radius * 0.90, height * 0.07, "Metal", height * 0.465, vertices=40))
            geometry.append(_cylinder("LidBand", radius * 0.94, height * 0.025, front_material, height * 0.475, vertices=40))
        else:
            geometry.append(_cylinder("TopRim", radius * 0.97, height * 0.035, "Metal", height * 0.4825, vertices=40))
            geometry.append(_cylinder("BottomRim", radius * 0.97, height * 0.025, "Metal", -height * 0.4875, vertices=40))
        geometry.append(asset_front_panel(width, depth, height * 0.65))
    elif spec.model_type == "bottle":
        radius = min(width, depth) / 2.0
        geometry.append(_cylinder("Body", radius * 0.91, height * 0.70, body_material, -height * 0.15, vertices=40))
        geometry.append(_frustum(
            "Shoulder", radius * 0.91, radius * 0.57,
            height * 0.095, body_material, height * 0.245,
        ))
        geometry.append(_cylinder("Neck", radius * 0.57, height * 0.17, body_material, height * 0.36, vertices=32))
        geometry.append(asset_front_panel(width, depth, height * 0.42))
        geometry.append(_cylinder("Cap", radius * 0.60, height * 0.05, "Metal", height * 0.475, vertices=32))
        geometry.append(_cylinder("CapRidge", radius * 0.63, height * 0.014, "Metal", height * 0.46, vertices=32))
    elif spec.model_type == "fruit":
        geometry.append(f'''        def Xform "Fruit" {{
            double3 xformOp:scale = ({width:.5f}, {depth:.5f}, {height:.5f})
            uniform token[] xformOpOrder = ["xformOp:scale"]
            def Sphere "Shape" (
                prepend apiSchemas = ["MaterialBindingAPI"]
            ) {{
                double radius = 0.5
                rel material:binding = </Asset/Looks/{body_material}>
            }}
        }}''')
        # Keep the small stalk inside the declared fruit bounds so shelf and
        # crate contact calculations include the entire authored mesh.
        geometry.append(_cylinder("Stem", width * 0.055, height * 0.10, "Stem", height * 0.45, vertices=12))
    elif spec.model_type == "crate":
        post = min(width, depth) * 0.05
        rail = min(width, depth) * 0.075
        geometry.append(_cube("Base", (width, depth, 0.025), (0.0, 0.0, -height / 2.0 + 0.0125), body_material))
        # Four corner posts and two open rows of rails make an actual bin
        # rather than a solid box with a fruit-colored sphere embedded in it.
        for ix, x in enumerate((-width / 2.0 + post / 2.0, width / 2.0 - post / 2.0)):
            for iy, y in enumerate((-depth / 2.0 + post / 2.0, depth / 2.0 - post / 2.0)):
                geometry.append(_cube(f"CornerPost{ix}{iy}", (post, post, height), (x, y, 0.0), body_material))
        # The lower rail row sits on the crate base perimeter, below the
        # bottom layer of fruit. Thin upper rails end flush with the posts;
        # their lower edge clears the packed upper fruit layer.
        for level, z in enumerate((-height / 2.0 + rail / 2.0, height / 2.0 - 0.008 / 2.0)):
            for side, y in enumerate((-depth / 2.0 + rail / 2.0, depth / 2.0 - rail / 2.0)):
                rail_height = rail if level == 0 else 0.008
                geometry.append(_cube(f"LongRail{level}{side}", (width - 2.0 * post, rail, rail_height), (0.0, y, z), front_material))
            for side, x in enumerate((-width / 2.0 + rail / 2.0, width / 2.0 - rail / 2.0)):
                rail_height = rail if level == 0 else 0.008
                geometry.append(_cube(f"EndRail{level}{side}", (rail, depth - 2.0 * post, rail_height), (x, 0.0, z), front_material))
    elif spec.model_type == "milk_jug":
        geometry.extend([
            _beveled_box("JugBody", (width, depth, height * 0.70), 0.018, "Body"),
            _cube("JugBase", (width * 0.94, depth * 0.94, height * 0.30), (0.0, 0.0, -height * 0.35), "Body"),
            _frustum("Shoulder", width * 0.48, width * 0.24, height * 0.16, "Body", height * 0.35, vertices=40),
            _cylinder_at("OffsetNeck", width * 0.115, height * 0.12, "Body", (width * 0.18, 0.0, height * 0.43), vertices=28),
            _cylinder_at("Cap", width * 0.13, height * 0.045, "Front", (width * 0.18, 0.0, height * 0.4775), vertices=28),
            _cube("HandleTop", (width * 0.34, depth * 0.15, height * 0.055), (-width * 0.22, 0.0, height * 0.36), "Body"),
            _cube("HandleSide", (width * 0.07, depth * 0.15, height * 0.26), (-width * 0.37, 0.0, height * 0.25), "Body"),
            asset_front_panel(width, depth, height * 0.48),
        ])
    elif spec.model_type == "gable_carton":
        geometry.extend([
            _beveled_box("CartonBody", (width, depth, height * 0.78), 0.006, "Paper"),
            _cube("CartonBase", (width * 0.98, depth * 0.98, height * 0.22), (0.0, 0.0, -height * 0.39), "Paper"),
            _wedge("GableLeft", (width * 0.50, depth, height * 0.22), "Front", (-width * 0.25, 0.0, height * 0.39)),
            _wedge("GableRight", (width * 0.50, depth, height * 0.22), "Paper", (width * 0.25, 0.0, height * 0.39)),
            _cube("TopSeam", (width * 0.16, depth, height * 0.035), (0.0, 0.0, height * 0.4825), "Front"),
            _cylinder_at("PourCap", width * 0.09, height * 0.03, "Metal", (width * 0.20, depth * 0.15, height * 0.43), vertices=24),
            asset_front_panel(width, depth, height * 0.70),
        ])
    elif spec.model_type == "yogurt_cup":
        geometry.extend([
            _frustum("TaperedCup", width * 0.41, width * 0.50, height * 0.84, "Body", -height * 0.04, vertices=40),
            _cylinder("FoilLid", width * 0.53, height * 0.025, "Metal", height * 0.4875, vertices=48),
            _cylinder("BaseRing", width * 0.39, height * 0.025, "Front", -height * 0.4875, vertices=40),
            asset_front_panel(width * 0.85, depth * 0.90, height * 0.55),
        ])
    elif spec.model_type == "egg_carton":
        geometry.extend([
            _beveled_box("Tray", (width, depth, height * 0.48), 0.012, "Paper"),
            _beveled_box("HingedLid", (width, depth * 0.94, height * 0.40), 0.016, "Body"),
            _cube("TrayFeet", (width * 0.94, depth * 0.90, height * 0.24), (0.0, 0.0, -height * 0.38), "Paper"),
            _cube("FrontLatch", (width * 0.12, depth * 0.06, height * 0.18), (0.0, depth * 0.48, -height * 0.02), "Front"),
        ])
        for index in range(6):
            x = -width * 0.405 + index * width * 0.162
            geometry.append(_ellipsoid(f"EggDome{index}", (width * 0.145, depth * 0.62, height * 0.70), (x, 0.0, height * 0.06), "Body"))
    elif spec.model_type == "butter_pack":
        geometry.extend([
            _beveled_box("WrappedBlock", spec.dimensions_m, 0.004, "Paper"),
            _cube("LeftFold", (width * 0.035, depth * 0.86, height * 0.78), (-width * 0.477, 0.0, 0.0), "Metal"),
            _cube("RightFold", (width * 0.035, depth * 0.86, height * 0.78), (width * 0.477, 0.0, 0.0), "Metal"),
            _cube("PaperBand", (width * 0.34, depth * 1.02, height * 0.90), (0.0, 0.0, 0.0), "Front"),
        ])
    elif spec.model_type == "cheese_wedge":
        geometry.extend([
            _wedge("Cheese", spec.dimensions_m, "Body"),
            _cube("WaxRind", (width * 0.04, depth, height), (-width * 0.48, 0.0, 0.0), "Front"),
            _cube("Label", (width * 0.42, 0.004, height * 0.52), (-width * 0.15, depth * 0.505, -height * 0.08), "Paper"),
        ])
    elif spec.model_type == "flat_carton":
        geometry.extend([
            _beveled_box("ShallowCarton", spec.dimensions_m, 0.012, "Paper"),
            _cube("EdgeLip", (width * 0.92, depth * 0.24, height * 0.92), (0.0, depth * 0.39, 0.0), "Front"),
            _cylinder_at("RoundWindow", width * 0.28, depth * 0.04, "Clear", (0.0, depth * 0.51, 0.0), axis="Y", vertices=48),
        ])
    elif spec.model_type == "oval_tub":
        geometry.extend([
            _oval_prism("OvalTub", (width * 0.96, depth * 0.96, height * 0.82), "Body", -height * 0.07, vertices=40),
            _oval_prism("TubFoot", (width * 0.82, depth * 0.82, height * 0.05), "Body", -height * 0.475, vertices=40),
            _oval_prism("SnapLid", (width, depth, height * 0.14), "Front", height * 0.43, vertices=40),
            _oval_prism("RolledRim", (width, depth * 0.96, height * 0.035), "Metal", height * 0.35, vertices=40),
            asset_front_panel(width * 0.82, depth, height * 0.48),
        ])
    elif spec.model_type in {"oil_bottle", "longneck_bottle"}:
        square = spec.model_type == "oil_bottle"
        body_height = height * (0.58 if square else 0.60)
        if square:
            geometry.append(_beveled_box("GlassBody", (width, depth, body_height), 0.010, "Glass"))
            geometry.append(_cube("GlassBase", (width * 0.94, depth * 0.94, height * 0.42), (0.0, 0.0, -height * 0.29), "Glass"))
        else:
            geometry.append(_cylinder("GlassBody", width * 0.48, body_height, "Glass", -height * 0.17, vertices=48))
            geometry.append(_cylinder("PuntRing", width * 0.30, height * 0.03, "Dark", -height * 0.485, vertices=36))
        geometry.extend([
            _frustum("LongShoulder", width * 0.46, width * 0.18, height * 0.17, "Glass", height * 0.21, vertices=40),
            _cylinder("LongNeck", width * 0.18, height * 0.23, "Glass", height * 0.37, vertices=32),
            _cylinder("Closure", width * (0.20 if square else 0.19), height * 0.045, "Metal", height * 0.4775, vertices=32),
            asset_front_panel(width, depth, height * 0.38),
        ])
    elif spec.model_type == "handled_bottle":
        geometry.extend([
            _beveled_box("FlaskBody", (width, depth, height * 0.76), 0.018, "Glass"),
            _cube("FlaskBase", (width * 0.94, depth * 0.94, height * 0.24), (0.0, 0.0, -height * 0.38), "Glass"),
            _cylinder_at("Neck", width * 0.13, height * 0.16, "Glass", (width * 0.16, 0.0, height * 0.40), vertices=28),
            _cylinder_at("Cap", width * 0.15, height * 0.045, "Metal", (width * 0.16, 0.0, height * 0.477), vertices=28),
            _cube("HandleTop", (width * 0.30, depth * 0.12, height * 0.045), (-width * 0.22, 0.0, height * 0.32), "Front"),
            _cube("HandleOuter", (width * 0.055, depth * 0.12, height * 0.23), (-width * 0.36, 0.0, height * 0.22), "Front"),
        ])
    elif spec.model_type == "grip_bottle":
        geometry.extend([
            _cylinder("LowerBody", width * 0.47, height * 0.60, "Body", -height * 0.20, vertices=40),
            _cylinder("GripWaist", width * 0.39, height * 0.18, "Body", height * 0.13, vertices=40),
            _frustum("Shoulder", width * 0.45, width * 0.23, height * 0.12, "Body", height * 0.28, vertices=40),
            _cylinder("SportsCap", width * 0.24, height * 0.16, "Front", height * 0.42, vertices=32),
        ])
        for index, z in enumerate((-0.30, -0.18, -0.06, 0.06)):
            geometry.append(_cylinder(f"GripRib{index}", width * 0.49, height * 0.018, "Front", height * z, vertices=40))
    elif spec.model_type in {"trigger_spray", "squeeze_bottle", "pump_bottle", "flip_bottle"}:
        profile = spec.model_type
        geometry.append(_beveled_box("BottleBody", (width, depth, height * 0.74), min(width, depth) * 0.20, "Body"))
        geometry.append(_cube("LowerBody", (width * 0.94, depth * 0.94, height * 0.26), (0.0, 0.0, -height * 0.37), "Body"))
        if profile == "trigger_spray":
            geometry.extend([
                _cylinder_at("OffsetNeck", width * 0.16, height * 0.14, "Body", (-width * 0.12, 0.0, height * 0.39), vertices=28),
                _cube("TriggerHead", (width * 0.72, depth * 0.82, height * 0.10), (width * 0.10, 0.0, height * 0.44), "Dark"),
                _cube("Nozzle", (width * 0.30, depth * 0.45, height * 0.07), (width * 0.35, 0.0, height * 0.46), "Front"),
                _wedge("TriggerLever", (width * 0.16, depth * 0.30, height * 0.24), "Dark", (width * 0.07, 0.0, height * 0.32)),
            ])
        elif profile == "pump_bottle":
            geometry.extend([
                _cylinder("Neck", width * 0.18, height * 0.12, "Body", height * 0.37, vertices=28),
                _cylinder("PumpStem", width * 0.07, height * 0.14, "Metal", height * 0.425, vertices=20),
                _cube("PumpHead", (width * 0.50, depth * 0.24, height * 0.045), (width * 0.12, 0.0, height * 0.475), "Dark"),
            ])
        elif profile == "flip_bottle":
            geometry.extend([
                _cube("BottomFlipCap", (width * 0.70, depth * 0.82, height * 0.09), (0.0, 0.0, -height * 0.455), "Front"),
                _cube("RecessedGrip", (width * 0.40, depth * 0.08, height * 0.19), (0.0, depth * 0.47, height * 0.12), "Dark"),
            ])
        else:
            geometry.extend([
                _frustum("Shoulder", width * 0.46, width * 0.20, height * 0.13, "Body", height * 0.31, vertices=32),
                _cylinder("FlipSpout", width * 0.20, height * 0.12, "Front", height * 0.44, vertices=28),
            ])
        geometry.append(asset_front_panel(width, depth, height * 0.48))
    elif spec.model_type in {"detergent_jug", "bleach_jug"}:
        bleach = spec.model_type == "bleach_jug"
        geometry.extend([
            _beveled_box("BroadJug", (width, depth, height * 0.76), 0.018, "Body"),
            _cube("JugBase", (width * 0.96, depth * 0.96, height * 0.24), (0.0, 0.0, -height * 0.38), "Body"),
            _frustum("UpperShoulder", width * 0.46, width * 0.22, height * 0.17, "Body", height * 0.34, vertices=40),
            _cylinder_at("OffsetSpout", width * 0.12, height * 0.12, "Body", ((-1 if bleach else 1) * width * 0.22, 0.0, height * 0.42), vertices=28),
            _cylinder_at("MeasuringCap", width * 0.15, height * 0.07, "Front", ((-1 if bleach else 1) * width * 0.22, 0.0, height * 0.465), vertices=28),
            _cube("HandleTop", (width * 0.36, depth * 0.13, height * 0.055), ((1 if bleach else -1) * width * 0.23, 0.0, height * 0.32), "Dark"),
            _cube("HandleSide", (width * 0.07, depth * 0.13, height * 0.25), ((1 if bleach else -1) * width * 0.38, 0.0, height * 0.22), "Dark"),
            asset_front_panel(width, depth, height * 0.46),
        ])
    elif spec.model_type in {"short_can", "canister"}:
        geometry.extend([
            _cylinder("CanBody", width * 0.48, height, "Body", vertices=48),
            _cylinder("TopRim", width * 0.50, height * 0.08, "Metal", height * 0.46, vertices=48),
            _cylinder("BottomRim", width * 0.50, height * 0.06, "Metal", -height * 0.47, vertices=48),
        ])
        if spec.model_type == "short_can":
            geometry.append(_ellipsoid("PullTab", (width * 0.28, depth * 0.12, height * 0.06), (width * 0.10, 0.0, height * 0.46), "Dark"))
        else:
            geometry.extend([
                _cylinder("Overcap", width * 0.51, height * 0.12, "Front", height * 0.44, vertices=48),
                _cube("GripBand", (width * 0.94, depth * 0.08, height * 0.08), (0.0, depth * 0.48, height * 0.36), "Dark"),
            ])
        geometry.append(asset_front_panel(width * 0.88, depth, height * 0.66))
    elif spec.model_type == "rectangular_tin":
        geometry.extend([
            _beveled_box("RoundedTin", spec.dimensions_m, 0.012, "Metal"),
            _cube("RolledSeam", (width * 0.94, depth * 0.94, height * 0.16), (0.0, 0.0, height * 0.42), "Front"),
            _ellipsoid("KeyTab", (width * 0.24, depth * 0.18, height * 0.08), (width * 0.24, 0.0, height * 0.43), "Dark"),
            asset_front_panel(width * 0.88, depth, height * 0.62),
        ])
    elif spec.model_type in {"hinged_box", "window_box", "long_box", "handled_box", "tissue_box"}:
        geometry.append(_beveled_box("Carton", spec.dimensions_m, min(width, depth) * 0.08, "Paper"))
        if spec.model_type == "hinged_box":
            geometry.extend([
                _cube("HingedLid", (width * 0.96, depth * 0.90, height * 0.16), (0.0, 0.0, height * 0.42), "Front"),
                _cube("FrontFlap", (width * 0.48, depth * 0.05, height * 0.28), (0.0, depth * 0.49, height * 0.22), "Dark"),
            ])
        elif spec.model_type == "window_box":
            geometry.extend([
                _cube("CellophaneWindow", (width * 0.58, 0.004, height * 0.43), (0.0, depth * 0.51, -height * 0.08), "Clear"),
                _cube("TopFlap", (width * 0.72, depth * 0.82, height * 0.04), (0.0, 0.0, height * 0.48), "Front"),
            ])
        elif spec.model_type == "long_box":
            geometry.extend([
                _cube("HingedFlap", (width * 0.88, depth * 0.08, height * 0.72), (0.0, depth * 0.48, height * 0.08), "Front"),
                _cube("SerratedEdge", (width * 0.94, depth * 0.05, height * 0.10), (0.0, depth * 0.52, -height * 0.36), "Metal"),
                _cylinder_at("FoilRoll", depth * 0.28, width * 0.84, "Metal", (0.0, 0.0, 0.0), axis="X", vertices=32),
            ])
        elif spec.model_type == "handled_box":
            geometry.extend([
                _cube("HandleLeft", (width * 0.08, depth * 0.10, height * 0.24), (-width * 0.20, 0.0, height * 0.38), "Dark"),
                _cube("HandleRight", (width * 0.08, depth * 0.10, height * 0.24), (width * 0.20, 0.0, height * 0.38), "Dark"),
                _cube("HandleTop", (width * 0.48, depth * 0.10, height * 0.06), (0.0, 0.0, height * 0.46), "Dark"),
                _cube("DispensingSlot", (width * 0.44, 0.005, height * 0.07), (0.0, depth * 0.51, -height * 0.28), "Clear"),
            ])
        else:
            geometry.extend([
                _ellipsoid("OvalOpening", (width * 0.42, depth * 0.28, height * 0.12), (0.0, 0.0, height * 0.43), "Dark"),
                _ellipsoid("RaisedTissue", (width * 0.36, depth * 0.20, height * 0.25), (0.0, 0.0, height * 0.36), "Paper"),
            ])
        geometry.append(asset_front_panel(width, depth, height * 0.72))
    elif spec.model_type in {"gusset_bag", "valve_bag", "paper_sack", "pillow_bag"}:
        geometry.extend(_pouch("Package", spec.dimensions_m, "Paper" if spec.model_type != "pillow_bag" else "Body"))
        if spec.model_type == "gusset_bag":
            geometry.extend([
                _cube("LeftGusset", (width * 0.12, depth * 0.94, height * 0.72), (-width * 0.40, 0.0, -height * 0.04), "Front"),
                _cube("RightGusset", (width * 0.12, depth * 0.94, height * 0.72), (width * 0.40, 0.0, -height * 0.04), "Front"),
            ])
        elif spec.model_type == "valve_bag":
            geometry.append(_cylinder_at("DegassingValve", width * 0.08, depth * 0.06, "Dark", (width * 0.22, depth * 0.50, height * 0.18), axis="Y", vertices=24))
        elif spec.model_type == "paper_sack":
            geometry.extend([
                _cube("PinchedTop", (width * 0.72, depth * 0.24, height * 0.10), (0.0, 0.0, height * 0.43), "Front"),
                _cube("FoldedBase", (width * 0.76, depth * 0.72, height * 0.07), (0.0, 0.0, -height * 0.465), "Front"),
            ])
        else:
            for side in (-1, 1):
                name = "CrimpBottom" if side < 0 else "CrimpTop"
                geometry.append(_cube(name, (width * 0.92, depth * 0.28, height * 0.045), (0.0, 0.0, side * height * 0.475), "Metal"))
        geometry.append(asset_front_panel(width * 0.90, depth, height * 0.62))
    elif spec.model_type == "bread_bag":
        geometry.extend([
            _loaf_mesh("ScoredLoaf", (width * 0.96, depth * 0.88, height * 0.76), "Paper"),
            _beveled_box("ClearSleeve", (width, depth, height * 0.78), 0.012, "Clear"),
            _cube("BagBottomSeal", (width * 0.84, depth * 0.52, height * 0.06), (0.0, 0.0, -height * 0.47), "Clear"),
            _frustum("TwistedNeck", width * 0.22, width * 0.10, height * 0.20, "Clear", height * 0.36, vertices=20),
            _cube("Closure", (width * 0.12, depth * 0.16, height * 0.04), (0.0, 0.0, height * 0.43), "Front"),
            asset_front_panel(width * 0.72, depth, height * 0.48),
        ])
    elif spec.model_type == "banana_bunch":
        for finger in range(5):
            geometry.append(_curved_banana_finger(f"Finger{finger}", spec.dimensions_m, finger, "Body"))
        geometry.extend([
            _ellipsoid("Crown", (width * 0.28, depth * 0.34, height * 0.22), (0.0, 0.0, height * 0.18), "Front"),
            _cylinder("Stem", width * 0.04, height * 0.24, "Stem", height * 0.37, vertices=14),
        ])
    elif spec.model_type == "pear":
        geometry.extend([
            _ellipsoid("Bulb", (width, depth, height * 0.70), (0.0, 0.0, -height * 0.15), "Body"),
            _frustum("TaperedTop", width * 0.38, width * 0.16, height * 0.30, "Body", height * 0.25, vertices=32),
            _cylinder("Stem", width * 0.04, height * 0.18, "Stem", height * 0.41, vertices=12),
            _wedge("Leaf", (width * 0.38, depth * 0.12, height * 0.16), "Stem", (width * 0.12, 0.0, height * 0.38)),
        ])
    elif spec.model_type == "broccoli":
        geometry.append(_frustum("Stalk", width * 0.20, width * 0.30, height * 0.55, "Stem", -height * 0.22, vertices=20))
        branch_centers = (
            (-width * .22, 0, height * .18), (width * .22, 0, height * .20),
            (0, depth * .20, height * .27), (-width * .32, -depth * .16, height * .32),
            (width * .32, -depth * .16, height * .32),
        )
        for index, (x, y, z) in enumerate(branch_centers):
            geometry.append(_frustum(f"Branch{index}", width * 0.075, width * 0.105, height * 0.34, "Stem", z - height * 0.18, vertices=14, translate_x=x, translate_y=y))
            for bud in range(3):
                geometry.append(_irregular_floret(
                    f"Floret{index}_{bud}",
                    (width * 0.25, depth * 0.24, height * 0.18),
                    (x + (bud - 1) * width * 0.075, y + (bud % 2) * depth * 0.055, z + (bud % 2) * height * 0.035),
                    "Body",
                ))
    elif spec.model_type == "carrot_bunch":
        for index, x in enumerate((-width * 0.26, 0.0, width * 0.26)):
            geometry.append(_frustum(f"Root{index}", width * 0.11, width * 0.025, height * 0.72, "Body", -height * 0.14, vertices=20, translate_x=x))
            geometry.append(_wedge(f"Leaf{index}", (width * 0.20, depth * 0.22, height * 0.30), "Stem", (x, 0.0, height * 0.34)))
        geometry.append(_cube("BindingBand", (width * 0.80, depth * 0.48, height * 0.055), (0.0, 0.0, height * 0.17), "Front"))
    elif spec.model_type in {"jam_jar", "spice_jar", "wide_jar"}:
        body_radius = min(width, depth) * (0.46 if spec.model_type != "wide_jar" else 0.48)
        vertices = 8 if spec.model_type == "jam_jar" else 32
        geometry.extend([
            _cylinder("JarBody", body_radius, height * 0.78, "Glass", -height * 0.08, vertices=vertices),
            _cylinder("BaseFoot", body_radius * 0.94, height * 0.06, "Glass", -height * 0.47, vertices=vertices),
            _frustum("Shoulder", body_radius, body_radius * 0.82, height * 0.12, "Glass", height * 0.33, vertices=vertices),
            _cylinder("TwistLid", body_radius * 0.88, height * 0.14, "Metal", height * 0.43, vertices=vertices),
        ])
        if spec.model_type == "spice_jar":
            geometry.append(_cylinder("ShakerInsert", body_radius * 0.70, height * 0.035, "Paper", height * 0.36, vertices=24))
        elif spec.model_type == "wide_jar":
            geometry.extend([
                _cube("LeftGrip", (width * 0.08, depth * 0.08, height * 0.34), (-width * 0.45, 0.0, -height * 0.08), "Front"),
                _cube("RightGrip", (width * 0.08, depth * 0.08, height * 0.34), (width * 0.45, 0.0, -height * 0.08), "Front"),
            ])
        geometry.append(asset_front_panel(width, depth, height * 0.48))
    elif spec.model_type == "paper_roll":
        geometry.extend([
            _cylinder("PaperCylinder", width * 0.50, height, "Paper", vertices=48),
            _cylinder("CardboardCore", width * 0.14, height, "Dark", vertices=32),
            _cylinder("Wrapper", width * 0.505, height * 0.88, "Clear", vertices=48),
        ])
        for index, z in enumerate((-0.30, -0.10, 0.10, 0.30)):
            geometry.append(_cylinder(f"EmbossBand{index}", width * 0.51, height * 0.012, "Front", height * z, vertices=48))
    elif spec.model_type == "blister_pack":
        geometry.extend([
            _beveled_box("CardBacker", (width, depth * 0.25, height), 0.006, "Paper"),
            _ellipsoid("ClearBlister", (width * 0.88, depth, height * 0.72), (0.0, depth * 0.10, -height * 0.06), "Clear"),
            _beveled_box("SpongeTop", (width * 0.72, depth * 0.42, height * 0.24), 0.010, "Body"),
            _beveled_box("SpongeBottom", (width * 0.72, depth * 0.42, height * 0.24), 0.010, "Front"),
            _ellipsoid("HangSlot", (width * 0.22, depth * 0.30, height * 0.07), (0.0, 0.0, height * 0.40), "Dark"),
        ])
    elif spec.model_type in {"angled_bin", "wicker_basket"}:
        if spec.model_type == "angled_bin":
            geometry.extend([
                _wedge("SlopedBase", (width, depth, height * 0.26), "Body", (0.0, 0.0, -height * 0.36)),
                _cube("BinFeet", (width * 0.92, depth * 0.84, height * 0.04), (0.0, 0.0, -height * 0.48), "Dark"),
                _cube("LowFront", (width, 0.025, height * 0.42), (0.0, depth * 0.46, -height * 0.22), "Front"),
                _cube("HighBack", (width, 0.025, height), (0.0, -depth * 0.46, 0.0), "Body"),
                _wedge("LeftCheek", (0.025, depth, height), "Body", (-width * 0.47, 0.0, 0.0)),
                _wedge("RightCheek", (0.025, depth, height), "Body", (width * 0.47, 0.0, 0.0)),
                _cube("FrontRail", (width * 0.88, 0.018, height * 0.08), (0.0, depth * 0.44, height * 0.02), "Front"),
                _cube("RearRail", (width * 0.88, 0.018, height * 0.08), (0.0, -depth * 0.44, height * 0.28), "Body"),
            ])
        else:
            geometry.append(_cube("WovenBase", (width, depth, 0.025), (0.0, 0.0, -height * 0.4375), "Body"))
            for index in range(8):
                geometry.append(_woven_strip(f"FrontWeave{index}", width, depth, height * 0.76, index, "Front" if index % 2 else "Body"))
                geometry.append(_woven_strip(f"BackWeave{index}", width, depth, height * 0.76, index, "Body" if index % 2 else "Front"))
            for index, x in enumerate((-width * .46, width * .46)):
                for side, y in enumerate((-depth * .45, depth * .45)):
                    geometry.append(_cube(f"CornerPost{index}{side}", (0.018, 0.018, height * 0.82), (x, y, 0.0), "Dark"))
            for index, y in enumerate((-depth * .45, -depth * .15, depth * .15, depth * .45)):
                geometry.append(_cube(f"CrossSlat{index}", (width, 0.012, height * 0.10), (0.0, y, -height * 0.18 + index * height * 0.12), "Body"))
            geometry.append(_cube("Rim", (width, depth, height * 0.06), (0.0, 0.0, height * 0.47), "Dark"))
    elif spec.model_type == "shelf_divider":
        geometry.extend([
            _cube("BaseClip", (width, depth, height * 0.10), (0.0, 0.0, -height * 0.45), "Metal"),
            _cube("VerticalFin", (width, depth * 0.92, height * 0.86), (0.0, 0.0, height * 0.02), "Clear"),
            _cube("FrontStop", (width * 1.8, depth * 0.06, height * 0.34), (0.0, depth * 0.45, -height * 0.30), "Front"),
        ])
    elif spec.model_type == "bottle_rack":
        geometry.append(_cube("RackBase", (width, depth, height * 0.18), (0.0, 0.0, -height * 0.41), "Dark"))
        for index, x in enumerate((-0.12, -0.04, 0.04, 0.12)):
            geometry.append(_cube(f"Channel{index}", (0.018, depth, height * 0.70), (x, 0.0, 0.0), "Metal"))
            geometry.append(_cube(f"FrontStop{index}", (0.055, depth * 0.05, height * 0.46), (x, depth * 0.47, -height * 0.18), "Front"))
        geometry.append(_cube("RearStop", (width, depth * 0.05, height * 0.82), (0.0, -depth * 0.47, -height * 0.05), "Metal"))
    elif spec.model_type == "price_display":
        geometry.extend([
            _cube("RailClip", (width, depth, height * 0.24), (0.0, -depth * 0.20, -height * 0.38), "Metal"),
            _beveled_box("Bezel", (width, depth, height), 0.006, "Dark"),
            _cube("EInkPanel", (width * 0.88, depth * 0.10, height * 0.68), (0.0, depth * 0.48, 0.0), "Paper"),
            asset_front_panel(width * 0.88, depth, height * 0.68),
        ])
    else:
        raise ValueError(spec.model_type)

    is_new_asset = bool(spec.assembly_parts)
    texture_exempt = spec.department == "produce" or spec.model_type in {
        "angled_bin", "wicker_basket", "shelf_divider", "bottle_rack",
    }
    if is_new_asset and not texture_exempt and not any("primvars:st" in fragment for fragment in geometry):
        geometry.append(asset_front_panel(width * 0.86, depth, height * 0.62))

    # Front is the one textured material and belongs only on explicit UV
    # meshes.  Colored caps, seams, rails, and closures use Accent instead.
    if is_new_asset:
        geometry = [
            fragment if "primvars:st" in fragment else fragment.replace("</Asset/Looks/Front>", "</Asset/Looks/Accent>")
            for fragment in geometry
        ]
        if (
            not is_hero_asset
            and any("primvars:st" in fragment for fragment in geometry)
            and spec.department not in {"produce", "fixtures"}
        ):
            geometry.extend(_side_and_back_print_panels(width, depth, height))
    if is_hero_asset:
        geometry.extend(_hero_side_and_back_print_panels(spec, "Print" if is_new_asset else "Front"))

    has_uv_label = any("primvars:st" in fragment for fragment in geometry)
    body_roughness = 0.62
    front_roughness = 0.54
    if is_new_asset or is_hero_asset:
        system = _new_art_system(spec)
        body_roughness = {
            "produce": 0.82,
            "fixture": 0.34,
            "household": 0.40,
            "chilled": 0.48,
            "bakery": 0.76,
            "bottle": 0.30,
            "wrap": 0.36,
            "soft_pack": 0.86,
            "carton": 0.74,
        }[system]
        front_roughness = {
            "produce": 0.80,
            "fixture": 0.30,
            "household": 0.42,
            "chilled": 0.52,
            "bakery": 0.66,
            "bottle": 0.38,
            "wrap": 0.48,
            "soft_pack": 0.72,
            "carton": 0.64,
        }[system]
    extended_materials = ""
    if spec.assembly_parts:
        extended_materials = "\n" + "\n".join((
            _material("Glass", (0.55, 0.68, 0.58), None, 0.16, 0.0, 0.72),
            _material("Paper", (0.82, 0.79, 0.69), None, 0.78, 0.0),
            _material("Clear", (0.74, 0.86, 0.91), None, 0.12, 0.0, 0.38),
            _material("Dark", (0.055, 0.065, 0.075), None, 0.38, 0.05),
            _material("Accent", spec.accent, None, front_roughness, 0.0),
            _material(
                "Print", spec.accent, texture_path if has_uv_label else None, front_roughness, 0.0, 1.0,
                normal_texture_path if has_uv_label else None,
                roughness_texture_path if has_uv_label else None,
            ),
        ))
    return f'''#usda 1.0
(
    defaultPrim = "Asset"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Asset" (
    kind = "component"
) {{
    def Xform "Geometry" {{
{chr(10).join(geometry)}
    }}
    def Scope "Looks" {{
{_material(body_material, spec.color, None, body_roughness, 0.0)}
{_material(
    front_material, spec.accent, texture_path if (has_uv_label or not is_new_asset) else None,
    front_roughness, 0.0, 1.0,
    normal_texture_path if has_uv_label else None,
    roughness_texture_path if has_uv_label else None,
)}
{_material("Metal", (0.48, 0.52, 0.55), None, 0.28, 0.7)}
{_material("Stem", (0.12, 0.23, 0.08), None, 0.72, 0.0)}{extended_materials}
    }}
}}
'''


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _usda_scope_sha256(path: Path, scope_name: str) -> str:
    """Hash one USDA scope so geometry and appearance remain separable."""
    text = path.read_text(encoding="utf-8")
    marker = f'def Xform "{scope_name}"' if scope_name == "Geometry" else f'def Scope "{scope_name}"'
    marker_index = text.find(marker)
    if marker_index < 0:
        raise ValueError(f"USD asset lacks {scope_name} scope: {path}")
    start = text.find("{", marker_index)
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                canonical = "".join(text[start + 1:index].split()).encode("utf-8")
                return hashlib.sha256(canonical).hexdigest()
    raise ValueError(f"USD asset has unbalanced {scope_name} scope: {path}")


def _geometry_signature(spec: AssetSpec) -> str:
    """Hash only physical construction evidence, never label or color data."""
    evidence = {
        "assembly_profile": spec.model_type,
        "assembly_parts": list(spec.assembly_parts or ("legacy_body", spec.model_type)),
        "dimensions_m": list(spec.dimensions_m),
        "support": spec.intended_support,
    }
    canonical = json.dumps(evidence, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _material_classes(spec: AssetSpec) -> tuple[str, ...]:
    if spec.model_type in {"can", "short_can", "canister", "rectangular_tin", "long_box"}:
        return ("printed_label", "metal")
    if spec.model_type in {"jar", "jam_jar", "spice_jar", "wide_jar", "oil_bottle", "longneck_bottle", "handled_bottle"}:
        return ("printed_label", "glass", "metal")
    if spec.model_type in {"fruit", "banana_bunch", "pear", "broccoli", "carrot_bunch"}:
        return ("produce_skin", "plant_stem")
    if spec.model_type in {"gusset_bag", "valve_bag", "paper_sack", "bread_bag", "pillow_bag", "blister_pack"}:
        return ("printed_film", "paper_or_product")
    if spec.model_type in {"crate", "angled_bin", "wicker_basket", "shelf_divider", "bottle_rack", "price_display"}:
        return ("fixture_body", "metal_or_clear_plastic")
    if spec.department in {"household", "refrigerated", "beverage"}:
        return ("printed_label", "molded_plastic")
    return ("printed_paperboard", "metal_or_plastic_detail")


def generate_library(root: Path | None = None) -> Path:
    _validate_texture_toolchain()
    repo = root or Path(__file__).resolve().parents[2]
    asset_root = repo / "assets" / "retail"
    usd_root = asset_root / "usd"
    texture_root = asset_root / "textures"
    usd_root.mkdir(parents=True, exist_ok=True)
    texture_root.mkdir(parents=True, exist_ok=True)
    entries = []
    for spec in ASSET_SPECS:
        texture_name = f"{spec.asset_key}.png"
        texture_path = texture_root / texture_name
        is_hero_asset = spec.asset_key in HERO_ART_DIRECTIONS
        candidate_normal_name = f"{spec.asset_key}_normal.png" if spec.assembly_parts or is_hero_asset else None
        candidate_roughness_name = f"{spec.asset_key}_roughness.png" if spec.assembly_parts or is_hero_asset else None
        usd_source = _asset_usda(spec, texture_name, candidate_normal_name, candidate_roughness_name)
        uses_surface_maps = 'def Shader "NormalTexture"' in usd_source
        normal_texture_name = candidate_normal_name if uses_surface_maps else None
        roughness_texture_name = candidate_roughness_name if uses_surface_maps else None
        normal_texture_path = texture_root / normal_texture_name if normal_texture_name else None
        roughness_texture_path = texture_root / roughness_texture_name if roughness_texture_name else None
        usd_path = usd_root / f"{spec.asset_key}.usda"
        if is_hero_asset:
            _write_hero_wrap_texture(texture_path, spec)
            assert normal_texture_path is not None and roughness_texture_path is not None
            _write_surface_maps(normal_texture_path, roughness_texture_path, spec)
        elif spec.assembly_parts:
            _write_category_texture(texture_path, spec)
            if uses_surface_maps:
                assert normal_texture_path is not None and roughness_texture_path is not None
                _write_surface_maps(normal_texture_path, roughness_texture_path, spec)
            else:
                # Organic produce and unprinted fixtures have no UV label
                # surface. Remove old generated maps instead of registering
                # source artifacts that no authored material consumes.
                assert candidate_normal_name is not None and candidate_roughness_name is not None
                (texture_root / candidate_normal_name).unlink(missing_ok=True)
                (texture_root / candidate_roughness_name).unlink(missing_ok=True)
        else:
            # Preserve the repaired baseline library byte-for-byte.
            _write_rich_texture(texture_path, spec)
        usd_path.write_text(usd_source, encoding="utf-8")
        entry = {
            "asset_key": spec.asset_key,
            "category": spec.category,
            # Paths are relative to this manifest.  Keeping them portable is
            # important because the same catalog is opened by Python, Isaac,
            # and an optional Blender source-authoring pass.
            "usd_path": f"usd/{spec.asset_key}.usda",
            "texture_path": f"textures/{texture_name}",
            "dimensions_m": list(spec.dimensions_m),
            "local_front_axis": "+Y",
            "model_type": spec.model_type,
            "product_name": spec.product_name,
            "department": spec.department,
            "intended_support": spec.intended_support,
            "assembly_profile": spec.model_type,
            "assembly_parts": list(spec.assembly_parts or ("legacy_body", spec.model_type)),
            "geometry_signature": _geometry_signature(spec),
            "material_classes": list(
                _material_classes(spec) + (("normal_map", "roughness_map") if uses_surface_maps else ())
            ),
            "introduced_in": "G02-A" if spec.assembly_parts else "baseline",
            "usd_sha256": _sha256(usd_path),
            "texture_sha256": _sha256(texture_path),
            "geometry_scope_sha256": _usda_scope_sha256(usd_path, "Geometry"),
            "appearance_scope_sha256": _usda_scope_sha256(usd_path, "Looks"),
        }
        if normal_texture_path is not None and roughness_texture_path is not None:
            entry.update({
                "normal_texture_path": f"textures/{normal_texture_name}",
                "normal_texture_sha256": _sha256(normal_texture_path),
                "roughness_texture_path": f"textures/{roughness_texture_name}",
                "roughness_texture_sha256": _sha256(roughness_texture_path),
            })
        if spec.asset_key in HERO_FOOD_SOURCES:
            source_name, source_hash = HERO_FOOD_SOURCES[spec.asset_key]
            entry.update({
                "source_food_path": f"source_food/{source_name}",
                "source_food_sha256": source_hash,
            })
        entries.append(entry)
    manifest = {
        "schema_version": 2,
        "standard_front_axis": "+Y",
        "units": "meters",
        "asset_register_path": "asset_register.json",
        "generation": {
            "generator": "tools/retail_assets/generate_packaging.py",
            "deterministic": True,
            "requirements": "tools/retail_assets/GENERATION_REQUIREMENTS.md",
            "pillow_version": REQUIRED_PILLOW_VERSION,
            "zlib_version": REQUIRED_ZLIB_VERSION,
            "font_sha256": {str(path): digest for path, digest in REQUIRED_FONTS.values()},
            "geometry_signature_excludes": ["asset_key", "product_name", "color", "accent", "texture"],
        },
        "assets": entries,
    }
    manifest_path = asset_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    new_entries = [entry for entry in entries if entry["introduced_in"] == "G02-A"]
    register = {
        "schema_version": 1,
        "provenance": {
            "provider": "project-authored deterministic procedural geometry",
            "source_url": None,
            "license": "project source; no third-party asset payload",
            "redistribution": "source and generated derivatives permitted with this repository",
            "generator": "tools/retail_assets/generate_packaging.py",
        },
        "audit": {
            "baseline_type_count": len(entries) - len(new_entries),
            "new_type_count": len(new_entries),
            "new_unique_geometry_signature_count": len({entry["geometry_signature"] for entry in new_entries}),
            "new_assembly_profiles": sorted({entry["assembly_profile"] for entry in new_entries}),
            "new_departments": sorted({entry["department"] for entry in new_entries}),
            "dimension_convention": "width_x, depth_y, height_z in meters",
            "bounds_tolerance_m": 0.012,
        },
        "assets": new_entries,
    }
    (asset_root / "asset_register.json").write_text(json.dumps(register, indent=2) + "\n", encoding="utf-8")
    return manifest_path


if __name__ == "__main__":
    print(generate_library())
