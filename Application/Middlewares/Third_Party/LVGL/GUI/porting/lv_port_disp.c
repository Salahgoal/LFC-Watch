#include "lv_port_disp.h" // LVGL显示端口对外接口
#include "lcd.h"          // ST7789底层显示驱动接口

/*
 * LVGL采用局部刷新模式，不需要在RAM中保存完整的240×280帧缓冲区。
 * 当前绘图缓冲区保存20行RGB565像素，占用空间为240×20×2=9600字节。
 *
 * 增大行数可以减少整屏刷新次数，但会占用更多RAM；减小则更节省RAM，
 * 但LVGL需要把同一画面拆成更多区域发送。20行是在刷新效率和RAM占用之间的折中。
 */
#define LVGL_DRAW_BUFFER_LINES 20U

/*
 * LVGL局部绘图缓冲区。LVGL先将控件和文字绘制到这里，再通过Flush回调发送给ST7789。
 * RGB565每个像素占2字节；使用uint8_t数组是为了匹配LVGL提供的像素数据指针类型。
 */
static uint8_t lvgl_draw_buffer[LVGL_DRAW_BUFFER_LINES * 240U * 2U];

/**
 * @brief 将LVGL绘制完成的局部区域刷新到ST7789
 *
 * LVGL显示刷新流程：
 *
 * LVGL计算需要重绘的区域
 *          ↓
 * 将RGB565像素绘制到lvgl_draw_buffer
 *          ↓
 * 调用LVGL_DisplayFlush()
 *          ↓
 * LCD_DrawImage()通过SPI/DMA发送像素
 *          ↓
 * lv_display_flush_ready()通知LVGL缓冲区可以再次使用
 *
 * 必须在像素传输真正完成后调用lv_display_flush_ready()。当前LCD_DrawImage()
 * 会等待DMA传输结束后才返回，因此可以在其后直接通知LVGL。如果以后改成完全异步DMA，
 * 则应把flush_ready调用移动到DMA完成回调中。
 *
 * @param display   发起本次刷新的LVGL显示设备
 * @param area      本次需要刷新的矩形坐标区域
 * @param pixel_map 指向该区域RGB565像素数据的指针
 */
static void LVGL_DisplayFlush(lv_display_t *display, const lv_area_t *area, uint8_t *pixel_map)
{
    uint16_t width;  // 刷新区域宽度
    uint16_t height; // 刷新区域高度

    /*
     * 只允许刷新完全位于屏幕内部的区域。非法区域不能传给LCD_DrawImage()，
     * 但仍需调用flush_ready，否则LVGL会一直等待本次刷新完成。
     */
    if(area->x1 < 0 || area->y1 < 0 || area->x2 >= (int32_t)LCD_WIDTH || area->y2 >= (int32_t)LCD_HEIGHT)
    {
        lv_display_flush_ready(display);
        return;
    }

    /*
     * LVGL的x2和y2包含最后一个像素，因此尺寸必须加1。
     * 例如x1=10、x2=19时，实际包含10~19共10个像素。
     */
    width = (uint16_t)(area->x2 - area->x1 + 1U);
    height = (uint16_t)(area->y2 - area->y1 + 1U);

    LCD_DrawImage((uint16_t)area->x1, (uint16_t)area->y1, width, height, pixel_map);

    lv_display_flush_ready(display); // 通知LVGL本次刷新完成，绘图缓冲区可以再次使用
}

/**
 * @brief 创建并注册LVGL显示设备
 *
 * 本函数负责建立LVGL与ST7789驱动之间的连接：
 * 1. 按LCD实际分辨率创建显示设备；
 * 2. 设置RGB565字节交换格式；
 * 3. 注册显示刷新回调；
 * 4. 注册局部绘图缓冲区；
 * 5. 将该设备设置为默认显示器。
 */
void LVGL_PortDisplay_Init(void)
{
    lv_display_t *display; // LVGL显示设备对象

    display = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

    /*
     * ST7789通过SPI按高字节、低字节顺序接收RGB565数据，
     * 因此使用RGB565_SWAPPED让LVGL提前交换每个像素的两个字节。
     */
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565_SWAPPED);

    lv_display_set_flush_cb(display, LVGL_DisplayFlush); // 注册LVGL到ST7789的刷新出口

    /*
     * 注册局部绘图缓冲区：
     * - display：当前显示设备；
     * - lvgl_draw_buffer：第一块绘图缓冲区；
     * - NULL：不使用第二块缓冲区；
     * - sizeof(...)：缓冲区总字节数；
     * - PARTIAL：只绘制并刷新发生变化的局部区域。
     */
    lv_display_set_buffers(display, lvgl_draw_buffer, NULL, sizeof(lvgl_draw_buffer), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_default(display); // 后续创建的页面和控件默认显示在该设备上
}
