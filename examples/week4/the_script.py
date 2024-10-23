from deform_plan.helpers.config_manager import ConfigManager
from deform_plan.helpers.seed_manager import init_manager
from parser_helper import parser
from maps import get_map

args = parser.parse_args()
cfg = ConfigManager({})
cfg.load_from_file(args.config)
init_manager(cfg.seed_env,cfg.seed_plan)
cur_map = get_map(args.map,cfg)
movable_idx = cur_map.get_movable_idx()


