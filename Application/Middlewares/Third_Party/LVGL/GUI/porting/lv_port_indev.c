#include "lv_port_indev.h" // LVGL触摸输入端口对外接口
#include "touch.h"         // CST816底层触摸驱动接口

/*
 * 保存最后一次有效触摸坐标。手指松开时，CST816可能不再提供有效坐标，
 * 但LVGL仍然需要知道手指是在哪个控件上释放的，因此松手后继续返回最后坐标。
 */
static int16_t touch_last_x = 0;
static int16_t touch_last_y = 0;

/*
 * 触摸唤醒状态变量：
 * - touch_wake_only：暗屏时设为1，触摸只用于唤醒，不传给LVGL控件。
 * - touch_wake_request：记录一次待处理的唤醒请求，读取后自动清零。
 * - touch_ignore_untill_release：亮屏后继续忽略当前手指，直到完全松手，防止唤醒触摸误点按钮。
 */
static uint8_t touch_wake_only = 0U;
static uint8_t touch_wake_request = 0U;
static uint8_t touch_ignore_untill_release = 0U;

static void LVGL_TouchRead(lv_indev_t *indev, lv_indev_data_t *data); // LVGL触摸读取回调

/**
 * @brief 创建并注册LVGL指针类型输入设备
 *
 * 初始化流程：
 * 1. 创建一个LVGL输入设备对象。
 * 2. 将设备类型设置为Pointer，表示鼠标或触摸屏一类的坐标输入设备。
 * 3. 注册LVGL_TouchRead()。之后LVGL会周期调用该函数读取触摸状态。
 */
void LVGL_PortInput_Init(void)
{
    lv_indev_t *touch_indev; // LVGL触摸输入设备对象

    touch_indev = lv_indev_create();
    if(touch_indev == NULL) return;

    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch_indev, LVGL_TouchRead);
}

/**
 * @brief 设置触摸输入是否只用于唤醒屏幕
 *
 * 暗屏时传入非0值，触摸只生成唤醒请求，不会继续传给LVGL控件；
 * 屏幕恢复后传入0，允许后续的新触摸正常操作界面。
 *
 * @param enable 0=普通触摸模式，非0=仅唤醒模式
 */
void LVGL_PortInput_SetWakeOnly(uint8_t enable)
{
    touch_wake_only = (enable != 0U) ? 1U : 0U; // 将任意非0参数统一转换为1
}

/**
 * @brief 读取并清除一次触摸唤醒请求
 *
 * 该函数采用“读取后清零”的一次性事件方式。UiTask每次读取后，
 * 同一次触摸不会被重复识别成多次唤醒请求。
 *
 * @return 1=检测到触摸唤醒请求，0=当前没有唤醒请求
 */
uint8_t LVGL_PortInput_TakeWakeRequest(void)
{
    uint8_t request = touch_wake_request; // 先保存请求状态，再清除全局标志

    touch_wake_request = 0U;

    return request;
}

/**
 * @brief 将CST816触摸状态转换为LVGL输入数据
 *
 * 这是LVGL注册的输入设备读取回调，由lv_timer_handler()周期调用。
 *
 * 正常触摸流程：
 * 1. 调用Touch_Update()读取CST816最新状态。
 * 2. 按下时保存坐标，并向LVGL返回PRESSED。
 * 3. 松开时返回RELEASED，同时保留最后一次有效坐标。
 *
 * 暗屏唤醒流程：
 * 1. touch_wake_only=1时，触摸只设置touch_wake_request。
 * 2. 始终向LVGL返回RELEASED，因此唤醒触摸不会点击界面控件。
 * 3. 屏幕恢复后继续等待手指完全松开。
 * 4. 手指松开后，下一次新的触摸才重新传给LVGL。
 *
 * @param indev 当前LVGL输入设备，本函数不需要直接使用
 * @param data  用于返回触摸坐标和按下/松开状态
 */
static void LVGL_TouchRead(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint8_t pressed; // 0=没有触摸，1=手指正在按下

    (void)indev; // 明确表示参数未使用，避免编译器产生警告

    Touch_Update(); // 从CST816读取并更新底层触摸状态

    pressed = (Touch_IsPressed() != 0U) ? 1U : 0U;

    /*
     * 暗屏期间进入仅唤醒模式。即使检测到手指按下，也始终向LVGL返回RELEASED，
     * 从而保证第一次触摸只唤醒屏幕，不会同时触发按钮或滑块。
     */
    if(touch_wake_only != 0U)
    {
        if(pressed != 0U)
        {
            touch_wake_request = 1U;          // 通知UiTask执行亮屏
            touch_ignore_untill_release = 1U; // 亮屏后继续等待本次手指松开
        }

        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = touch_last_x;
        data->point.y = touch_last_y;
        return;
    }

    /*
     * 屏幕虽然已经恢复亮度，但用于唤醒的手指可能仍按在屏幕上。
     * 在检测到手指完全松开前继续向LVGL返回RELEASED，避免同一次触摸在亮屏后
     * 又被LVGL解释成一次按钮点击。松开后清除标志，下一次触摸恢复正常处理。
     */
    if(touch_ignore_untill_release != 0U)
    {
        if(pressed == 0U) touch_ignore_untill_release = 0U;

        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = touch_last_x;
        data->point.y = touch_last_y;
        return;
    }

    if(pressed != 0U)
    {
        touch_last_x = Touch_GetX(); // 只在按下时保存有效X坐标
        touch_last_y = Touch_GetY(); // 只在按下时保存有效Y坐标
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    /* 无论按下还是松开，都向LVGL提供最后一次有效坐标。 */
    data->point.x = touch_last_x;
    data->point.y = touch_last_y;
}
