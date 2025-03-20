import gradio as gr
import krpc
from threading import Thread, Lock
from Internal import Targets, Targets_JNSQ, KAL
import locale

# 检测系统语言
system_language = locale.getdefaultlocale()[0]

# 多语言字典
translations = {
    "en": {
        "KAL_console": "K.A.L. Console",
        "select_rocket_and_target": "Select Rocket and Target Landing Zone",
        "rocket_name": "Rocket Name",
        "target_zone": "Target Zone",
        "parameters": "Parameters",
        "skip_boosterback": "Skip Boosterback",
        "skip_entryburn": "Skip Entry Burn",
        "skip_aerodynamic_guidance": "Skip Aerodynamic Guidance",
        "max_tilt_angle": "Max Tilt Angle",
        "throttle_limit_min": "Throttle Limit Min",
        "throttle_limit_max": "Throttle Limit Max",
        "target_roll_angle": "Target Roll Angle",
        "gfold_start_velocity": "G-Fold Start Velocity",
        "land_confirm": "Land Confirm",
        "deploy_landing_gear": "Deploy Landing Gear",
        "final_altitude": "Final Altitude (Set to 0 for auto)",
        "start_landing": "Start Landing",
        "landing_status": "Landing Status",
        "error_recovering": "Error: {vessel_name} is already recovering!",
        "refresh_vessels": "Refresh Vessels"  # 新增翻译
    },
    "zh": {
        "KAL_console": "K.A.L. 控制台",
        "select_rocket_and_target": "选择火箭和目标区域",
        "rocket_name": "火箭名称",
        "target_zone": "目标区域",
        "parameters": "参数配置",
        "skip_boosterback": "跳过返场机动",
        "skip_entryburn": "跳过减速燃烧",
        "skip_aerodynamic_guidance": "跳过气动引导",
        "max_tilt_angle": "最大倾斜角度",
        "throttle_limit_min": "节流阀下限",
        "throttle_limit_max": "节流阀上限",
        "target_roll_angle": "目标滚转角度",
        "gfold_start_velocity": "G-Fold 启动速度",
        "land_confirm": "着陆确认",
        "deploy_landing_gear": "展开起落架",
        "final_altitude": "最终高度（设置为0自动决定）",
        "start_landing": "开始着陆",
        "landing_status": "着陆状态",
        "error_recovering": "错误：{vessel_name} 已经在回收中！",
        "refresh_vessels": "刷新可用载具"  # 新增翻译
    }
}


# 根据系统语言选择翻译
lang = "zh" if system_language.startswith("zh") else "en"

# 连接到 KSP
conn = krpc.connect(name='KAL')
space_center = conn.space_center

# 工具函数
def get_all_available_vessel():
    return [vessel.name for vessel in space_center.vessels]

def check_JNSQ():
    return space_center.bodies['Kerbin'].atmosphere_depth > 70000

def get_targets():
    if check_JNSQ():
        return Targets_JNSQ
    else:
        return Targets

# 用于存储正在回收的载具名称
recovering_vessels = []

# GUI 函数
def start_landing(vessel_name, target_zone, skip_boosterback, skip_entryburn, skip_aerodynamic_guidance, max_tilt, throttle_limit_min, throttle_limit_max, target_roll, gfold_start_velocity, land_confirm, landing_gear, final_altitude):
    # 检查是否已经选择过该载具
    if vessel_name in recovering_vessels:
        return translations[lang]["error_recovering"].format(vessel_name=vessel_name), gr.Dropdown.update(choices=update_vessel_list())
    
    # 获取目标区域
    target = getattr(get_targets(), target_zone)
    lock = Lock()
    # 创建 KAL 对象
    params = {
        'skip_boosterback': skip_boosterback,
        'skip_entryburn': skip_entryburn,
        'skip_aerodynamic_guidance': skip_aerodynamic_guidance,
        'max_tilt': max_tilt,
        'throttle_limit': [throttle_limit_min, throttle_limit_max],
        'target_roll': target_roll,
        'gfold_start_velocity': gfold_start_velocity,
        'land_confirm': land_confirm,
        'landing_gear': landing_gear,
        'final_altitude': final_altitude
    }
    kal = KAL(vessel_name, target, params, lock=lock)
    # 启动着陆线程
    landing_thread = Thread(target=kal.land)
    landing_thread.start()
    
    # 将载具名称添加到正在回收的集合中
    recovering_vessels.append(vessel_name)
    
    return f"{translations[lang]['start_landing']}: {vessel_name} -> {target_zone}", gr.Dropdown(choices=update_vessel_list())

# 更新可用载具列表
def update_vessel_list():
    all_vessels = get_all_available_vessel()
    available_vessels = [vessel for vessel in all_vessels if vessel not in recovering_vessels]
    return available_vessels

# GUI 界面
with gr.Blocks() as demo:
    gr.Markdown(f"# {translations[lang]['KAL_console']}")
    gr.Markdown(f"## {translations[lang]['select_rocket_and_target']}")
    vessel_name = gr.Dropdown(choices=get_all_available_vessel(), label=translations[lang]['rocket_name'])
    target_zone = gr.Dropdown(choices=[zone for zone in dir(get_targets()) if not zone.startswith('_')], label=translations[lang]['target_zone'])

    gr.Markdown(f"## {translations[lang]['parameters']}")
    with gr.Column():
        skip_boosterback = gr.Checkbox(label=translations[lang]['skip_boosterback'], value=False)
        skip_entryburn = gr.Checkbox(label=translations[lang]['skip_entryburn'], value=False)
        skip_aerodynamic_guidance = gr.Checkbox(label=translations[lang]['skip_aerodynamic_guidance'], value=False)
        max_tilt = gr.Slider(label=translations[lang]['max_tilt_angle'], minimum=0, maximum=90, step=1, value=20)
        throttle_limit_min = gr.Slider(label=translations[lang]['throttle_limit_min'], minimum=0, maximum=1, step=0.01, value=0.1)
        throttle_limit_max = gr.Slider(label=translations[lang]['throttle_limit_max'], minimum=0, maximum=1, step=0.01, value=1)
        target_roll = gr.Slider(label=translations[lang]['target_roll_angle'], minimum=-180, maximum=180, step=1, value=0)
        gfold_start_velocity = gr.Slider(label=translations[lang]['gfold_start_velocity'], minimum=0, maximum=200, step=1, value=140)
        land_confirm = gr.Checkbox(label=translations[lang]['land_confirm'], value=True)
        landing_gear = gr.Checkbox(label=translations[lang]['deploy_landing_gear'], value=True)
        final_altitude = gr.Slider(label=translations[lang]['final_altitude'], minimum=0, maximum=100, step=1, value=0)

    start_button = gr.Button(translations[lang]['start_landing'])
    output = gr.Textbox(label=translations[lang]['landing_status'])

    # 点击开始着陆按钮时触发的事件
    start_button.click(
        fn=start_landing,
        inputs=[
            vessel_name, target_zone,
            skip_boosterback, skip_entryburn, skip_aerodynamic_guidance,
            max_tilt, throttle_limit_min, throttle_limit_max,
            target_roll, gfold_start_velocity,
            land_confirm, landing_gear, final_altitude
        ],
        outputs=[output, vessel_name]
    )

# 启动 Gradio 应用
demo.launch()