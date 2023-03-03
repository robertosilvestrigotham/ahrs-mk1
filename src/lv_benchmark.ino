#include "Wire.h"
#include "XL9535_driver.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include <lvgl.h>
#include "pin_config.h"
#include <Arduino.h>

const lv_color_t BKG = lv_color_hex(0x1E1E1E);
const lv_color_t WHITE = lv_color_hex(0xFFFFFF);
const lv_color_t BLACK = lv_color_hex(0x000000);
const lv_color_t CYAN = lv_color_hex(0x0B9CFF);
const lv_color_t BROWN = lv_color_hex(0xB26714);
const lv_color_t MAGENTA = lv_color_hex(0xE700A2);
const lv_color_t GREYTEXT = lv_color_hex(0x888888);
const lv_color_t YELLOW = lv_color_hex(0xE7E600);
const lv_color_t GREEN = lv_color_hex(0x00FF00);
bool chipTarget = 1;


short int roll = 5;
short int pitch = 0;




#define TOUCH_MODULE_CST820
// #define TOUCH_MODULE_FT3267

#if defined(TOUCH_MODULE_FT3267)
#include "ft3267.h"
#elif defined(TOUCH_MODULE_CST820)
#define TOUCH_MODULES_CST_SELF
#include "TouchLib.h"
#endif

typedef struct {
  uint8_t cmd;
  uint8_t data[16];
  uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3b, 0x00}, 0x02},
    {0xC1, {0x0b, 0x02}, 0x02},
    {0xC2, {0x07, 0x02}, 0x02},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01}, // 用565时屏蔽    666打开
    {0xb0, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 0x10},
    {0xb1, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13, 0x11, 0x2a, 0x31, 0x18}, 0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x6d}, 0x01},
    {0xb1, {0x37}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x43}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1, {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20}, 0x0b},
    {0xe2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x0d},
    {0xe3, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe4, {0x22, 0x00}, 0x02},
    {0xe5, {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe7, {0x22, 0x00}, 0x02},
    {0xe8, {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 0x07},
    {0xed, {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0, 0xab, 0xff, 0xff, 0xff}, 0x10},
    {0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xef, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},
    {0x36, {0x08}, 0x01},
    {0x3a, {0x66}, 0x01},
    {0x11, {0x00}, 0x80},
    // {0xFF, {0x77, 0x01, 0x00, 0x00, 0x12}, 0x05},
    // {0xd1, {0x81}, 0x01},
    // {0xd2, {0x06}, 0x01},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}};

XL9535 xl;
#if defined(TOUCH_MODULE_CST820)
TouchLib touch(Wire, IIC_SDA_PIN, IIC_SCL_PIN, CTS820_SLAVE_ADDRESS);
#endif

bool touch_pin_get_int = false;
void tft_init(void);
void lcd_cmd(const uint8_t cmd);
void lcd_data(const uint8_t *data, int len);


static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int16_t offsetx1 = area->x1;
  int16_t offsetx2 = area->x2;
  int16_t offsety1 = area->y1;
  int16_t offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {

  if (touch_pin_get_int) {

#if defined(TOUCH_MODULE_FT3267)
    uint8_t touch_points_num;
    uint16_t x, y;
    ft3267_read_pos(&touch_points_num, &x, &y);
    data->point.x = x;
    data->point.y = y;
#elif defined(TOUCH_MODULE_CST820)
    touch.read();
    TP_Point t = touch.getPoint(0);
    data->point.x = t.x;
    data->point.y = t.y;
#endif
    data->state = LV_INDEV_STATE_PR;
    touch_pin_get_int = false;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void preSetup(){
  static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;      // contains callback functions
  static lv_indev_drv_t indev_drv;
  // put your setup code here, to run once:
  pinMode(BAT_VOLT_PIN, ANALOG);

  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t)400000);
  Serial.begin(115200);
  xl.begin();
  uint8_t pin = (1 << PWR_EN_PIN) | (1 << LCD_CS_PIN) | (1 << TP_RES_PIN) | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) |
                (1 << LCD_RST_PIN) | (1 << SD_CS_PIN);

  xl.pinMode8(0, pin, OUTPUT);
  xl.digitalWrite(PWR_EN_PIN, 1);
  pinMode(EXAMPLE_PIN_NUM_BK_LIGHT, OUTPUT);
  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

  xl.digitalWrite(TP_RES_PIN, 0);
  delay(200);
  xl.digitalWrite(TP_RES_PIN, 1);
  
#if defined(TOUCH_MODULE_FT3267)
  ft3267_init(Wire);
#elif defined(TOUCH_MODULE_CST820)
  touch.init();
#endif
  tft_init();
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
      .clk_src = LCD_CLK_SRC_PLL160M,
      .timings =
          {
              .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
              .h_res = LCD_H_RES,
              .v_res = LCD_V_RES,
              // The following parameters should refer to LCD spec
              .hsync_pulse_width = 1,
              .hsync_back_porch = 30,
              .hsync_front_porch = 50,
              .vsync_pulse_width = 1,
              .vsync_back_porch = 30,
              .vsync_front_porch = 20,
              .flags =
                  {
                      .pclk_active_neg = 1,
                  },
          },
      .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
      .psram_trans_align = 64,
      .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
      .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
      .de_gpio_num = EXAMPLE_PIN_NUM_DE,
      .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
      .data_gpio_nums =
          {
              // EXAMPLE_PIN_NUM_DATA0,
              EXAMPLE_PIN_NUM_DATA13,
              EXAMPLE_PIN_NUM_DATA14,
              EXAMPLE_PIN_NUM_DATA15,
              EXAMPLE_PIN_NUM_DATA16,
              EXAMPLE_PIN_NUM_DATA17,

              EXAMPLE_PIN_NUM_DATA6,
              EXAMPLE_PIN_NUM_DATA7,
              EXAMPLE_PIN_NUM_DATA8,
              EXAMPLE_PIN_NUM_DATA9,
              EXAMPLE_PIN_NUM_DATA10,
              EXAMPLE_PIN_NUM_DATA11,
              // EXAMPLE_PIN_NUM_DATA12,

              EXAMPLE_PIN_NUM_DATA1,
              EXAMPLE_PIN_NUM_DATA2,
              EXAMPLE_PIN_NUM_DATA3,
              EXAMPLE_PIN_NUM_DATA4,
              EXAMPLE_PIN_NUM_DATA5,
          },
      .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
      .on_frame_trans_done = NULL,
      .user_ctx = NULL,
      .flags =
          {
              .fb_in_psram = 1, // allocate frame buffer in PSRAM
          },
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  lv_init();
  // alloc draw buffers used by LVGL from PSRAM
  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  pinMode(TP_INT_PIN, INPUT_PULLUP);
  attachInterrupt(
      TP_INT_PIN, [] { touch_pin_get_int = true; }, FALLING);
}


lv_obj_t *canvas;
void createCanvas(){
  canvas = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(canvas);
  lv_obj_set_size(canvas,LCD_H_RES,LCD_V_RES);
  lv_obj_set_style_bg_opa(canvas,LV_OPA_COVER,0);
  lv_obj_set_style_bg_color(canvas,CYAN,0);
}


lv_img_dsc_t *horizonCanvasDSC;
const short int horizonCanvasHeight = 330;
const short int horizonCanvasWidth = 280;
const short int horizonCanvasHalfHeight = horizonCanvasHeight/2;
const short int horizonCanvasHalfWidth = horizonCanvasWidth/2;
lv_color_t updateHorizonC0;
lv_color_t updateHorizonC1;
lv_color_t updateHorizonC2;
static lv_color_t horizonCanvasBuf[LV_CANVAS_BUF_SIZE_INDEXED_2BIT(horizonCanvasWidth, horizonCanvasHeight)];
lv_obj_t *horizonCanvas;
lv_obj_t *horizon;
lv_draw_rect_dsc_t horizonDsc;
lv_point_t horizonPoints[4];
const int8_t horizonPointsCount=4;
void createHorizon(){
  horizon = lv_obj_create(canvas);
  lv_obj_remove_style_all(horizon);
  lv_obj_set_pos(horizon,0,0);
  lv_obj_set_size(horizon,LCD_H_RES,LCD_V_RES);

  horizonCanvas = lv_canvas_create(horizon);
  lv_canvas_set_buffer(horizonCanvas, horizonCanvasBuf, horizonCanvasWidth, horizonCanvasHeight, LV_IMG_CF_INDEXED_2BIT);
  lv_obj_set_size(horizonCanvas,horizonCanvasWidth,horizonCanvasHeight);
  lv_obj_set_align(horizonCanvas,LV_ALIGN_CENTER);
  lv_canvas_set_palette(horizonCanvas, 0, BLACK);
  lv_canvas_set_palette(horizonCanvas, 1, CYAN);
  lv_canvas_set_palette(horizonCanvas, 2, BROWN);
  lv_canvas_set_palette(horizonCanvas, 3, WHITE);
  updateHorizonC0.full = 1;
  updateHorizonC1.full = 2;
  updateHorizonC2.full = 3;
  lv_canvas_fill_bg(horizonCanvas,updateHorizonC0,LV_OPA_COVER);

  lv_draw_rect_dsc_init(&horizonDsc);
  horizonDsc.bg_color = updateHorizonC1;
  
  calculatePolygonPoints();
  horizonCanvasDSC = lv_canvas_get_img(horizonCanvas);
  draw_polygon(updateHorizonC1);
}

const lv_point_t updateHorizonCenter = {horizonCanvasHalfWidth,horizonCanvasHalfHeight};
const short int updateHorizonCenterSizes[11] = {250,30,50,30,100,30,50,30,100,30,50};
lv_point_t updateHorizonStartPoint;
lv_point_t updateHorizonEndPoint;
void updateHorizon(){
    calculatePolygonPoints();
    lv_canvas_fill_bg(horizonCanvas, updateHorizonC0, LV_OPA_COVER);
    draw_polygon(updateHorizonC1);
    for(int8_t i=0;i<11;i++){
      get_line_points(updateHorizonCenter,roll,pitch + 12*i,updateHorizonCenterSizes[i],&updateHorizonStartPoint,&updateHorizonEndPoint);
      if(i%4==0){ 
        draw_line(horizonCanvas,updateHorizonStartPoint,updateHorizonEndPoint,updateHorizonC2);
      }else{
        draw_line_one(horizonCanvas,updateHorizonStartPoint,updateHorizonEndPoint,updateHorizonC2);
      }
      get_line_points(updateHorizonCenter,roll+180,-pitch + 12*i,updateHorizonCenterSizes[i],&updateHorizonStartPoint,&updateHorizonEndPoint);
      if(i>0 && i%4==0){ 
        draw_line(horizonCanvas,updateHorizonStartPoint,updateHorizonEndPoint,updateHorizonC2);
      }else{
        draw_line_one(horizonCanvas,updateHorizonStartPoint,updateHorizonEndPoint,updateHorizonC2);
      }
    }
}
int16_t draw_polygon_min_x = horizonCanvasWidth;
int16_t draw_polygon_min_y = horizonCanvasHeight;
int16_t draw_polygon_max_x = 0;
int16_t draw_polygon_max_y = 0;
void draw_polygon(lv_color_t color) {
  int8_t i = 0; 
  while (i < 4) {
    if (horizonPoints[i].x < draw_polygon_min_x) {
      draw_polygon_min_x = horizonPoints[i].x;
    }
    if (horizonPoints[i].x > draw_polygon_max_x) {
      draw_polygon_max_x = horizonPoints[i].x;
    }
    if (horizonPoints[i].y < draw_polygon_min_y) {
      draw_polygon_min_y = horizonPoints[i].y;
    }
    if (horizonPoints[i].y > draw_polygon_max_y) {
      draw_polygon_max_y = horizonPoints[i].y;
    }
    i++;
  }
  for (int16_t x = draw_polygon_min_x; x <= draw_polygon_max_x; x++) {
    for (int16_t y = draw_polygon_min_y; y <= draw_polygon_max_y; y++) {
      int16_t i, j;
      bool inside = false;
      for (i = 0, j = 3; i < 4; j = i++) {
        if (((horizonPoints[i].y >= y) != (horizonPoints[j].y >= y)) &&
            (x < (horizonPoints[j].x - horizonPoints[i].x) * (y - horizonPoints[i].y) / (horizonPoints[j].y - horizonPoints[i].y) + horizonPoints[i].x)) {
          inside = !inside;
        }
      }
      if (inside) {
        lv_img_buf_set_px_color(horizonCanvasDSC, x, y, color);
      }
    }
  }
}
const float radianCoef = M_PI / 180.0;
void calculatePolygonPoints() {
  const float tanValue = tan(-roll * radianCoef);
  // Calcolo le coordinate dei punti del poligono in base all'angolo
  horizonPoints[0].x = 0;
  horizonPoints[0].y = -pitch + horizonCanvasHalfHeight + int(horizonCanvasHalfWidth * tanValue);
  horizonPoints[1].x = horizonCanvasWidth;
  horizonPoints[1].y = -pitch + horizonCanvasHalfHeight - int(horizonCanvasHalfWidth * tanValue);
  horizonPoints[2].x = horizonCanvasWidth;
  horizonPoints[2].y = horizonCanvasHeight;
  horizonPoints[3].x = 0;
  horizonPoints[3].y = horizonCanvasHeight;
}
void draw_line(lv_obj_t *canvas, lv_point_t point_a, lv_point_t point_b, lv_color_t color) {
  LV_ASSERT_OBJ(canvas, MY_CLASS);
  lv_img_dsc_t *dsc = lv_canvas_get_img(canvas);

  int16_t x0 = point_a.x;
  int16_t y0 = point_a.y;
  int16_t x1 = point_b.x;
  int16_t y1 = point_b.y;

  int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int16_t dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int16_t err = (dx > dy ? dx : -dy) / 2, e2;

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (i == 0 && j == 0) {
        continue;
      }
      int16_t px0 = x0 + i;
      int16_t py0 = y0 + j;
      int16_t px1 = x1 + i;
      int16_t py1 = y1 + j;

      int16_t dx = abs(px1 - px0), sx = px0 < px1 ? 1 : -1;
      int16_t dy = abs(py1 - py0), sy = py0 < py1 ? 1 : -1;
      int16_t err = (dx > dy ? dx : -dy) / 2, e2;

      while (true) {
        if(px0>=0 && py0>=0 && px0<=horizonCanvasWidth && py0<=horizonCanvasHeight){
          lv_img_buf_set_px_color(dsc, px0, py0, color);
        }
        if (px0 == px1 && py0 == py1) {
          break;
        }
        e2 = err;
        if (e2 > -dx) {
          err -= dy;
          px0 += sx;
        }
        if (e2 < dy) {
          err += dx;
          py0 += sy;
        }
      }
    }
  }
}
void draw_line_one(lv_obj_t *canvas, lv_point_t point_a, lv_point_t point_b, lv_color_t color) {
  LV_ASSERT_OBJ(canvas, MY_CLASS);
  lv_img_dsc_t *dsc = lv_canvas_get_img(canvas);

  int16_t x0 = point_a.x;
  int16_t y0 = point_a.y;
  int16_t x1 = point_b.x;
  int16_t y1 = point_b.y;

  int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int16_t dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int16_t err = (dx > dy ? dx : -dy) / 2, e2;

  while (true) {
    if(x0>=0 && y0>=0 && x0<=horizonCanvasWidth && y0<=horizonCanvasHeight){
      lv_img_buf_set_px_color(dsc, x0, y0, color);
    }
    if (x0 == x1 && y0 == y1) {
      break;
    }
    e2 = err;
    if (e2 > -dx) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dy) {
      err += dx;
      y0 += sy;
    }
  }
}
void get_line_points(lv_point_t center, int16_t angle_degrees, int16_t distance, int16_t width, lv_point_t *start_point, lv_point_t *end_point) {
  lv_point_t centerPoint;
  if(distance!=0){
    const float centerTeta = (angle_degrees-90) * radianCoef;
    const float centerXPolar = distance * cos(centerTeta);
    const float centerYPolar = distance * sin(centerTeta);
    const float centerXCartesian = centerXPolar * cos(0) - centerYPolar * sin(0);
    const float centerYCartesian = centerXPolar * sin(0) + centerYPolar * cos(0);
    const int16_t centerX = center.x + int(centerXCartesian);
    const int16_t centerY = center.y + int(centerYCartesian);
    centerPoint = {centerX,centerY};
  }else{
    centerPoint = center;
  }
  const float theta = angle_degrees * M_PI / 180.0;
  const float xPolar = width/2 * cos(theta);
  const float yPolar = width/2 * sin(theta);
  const float xCartesian = xPolar * cos(0) - yPolar * sin(0);
  const float yCartesian = xPolar * sin(0) + yPolar * cos(0);
  int16_t x = centerPoint.x + int(xCartesian);
  int16_t y = centerPoint.y + int(yCartesian);
  *start_point = (lv_point_t){x, y};
  x = centerPoint.x - int(xCartesian);
  y = centerPoint.y - int(yCartesian);
  *end_point = (lv_point_t){x, y};
}


lv_obj_t *hdgObj;
lv_obj_t *hdgUnitObj;
lv_obj_t *hdgValueObj;
int16_t hdgValue = 245;

void createHDG(){
  hdgObj = lv_obj_create(canvas);
  lv_obj_remove_style_all(hdgObj);
  lv_obj_set_size(hdgObj,386,77);
  lv_obj_set_align(hdgObj,LV_ALIGN_BOTTOM_MID);  
  lv_obj_set_style_bg_opa(hdgObj,LV_OPA_COVER,0);
  lv_obj_set_style_bg_color(hdgObj,BKG,0);

  lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.radius = 0;
    rect_dsc.bg_opa = LV_OPA_COVER;
    rect_dsc.bg_color = YELLOW;

  static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(80, 40)];
    lv_obj_t * triangle = lv_canvas_create(canvas);
    lv_canvas_set_buffer(triangle, cbuf, 80, 40, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_set_pos(triangle,200,390);
    lv_canvas_draw_rect(triangle,0,0,80,30,&rect_dsc);
    const lv_point_t point[]={
      {50,30},
      {40,40},
      {30,30}
    };
    lv_canvas_draw_polygon(triangle,point,3,&rect_dsc);

  hdgUnitObj = lv_label_create(canvas);
    lv_label_set_text(hdgUnitObj,"m");
    lv_obj_set_pos(hdgUnitObj,260,403);
    lv_obj_set_style_text_color(hdgUnitObj,BLACK,0);
    lv_obj_set_style_text_font(hdgUnitObj, &lv_font_montserrat_12, 0);

  lv_obj_t *hdgUnitDeg = lv_label_create(canvas);
    lv_label_set_text(hdgUnitDeg,"°");
    lv_obj_set_pos(hdgUnitDeg,258,393);
    lv_obj_set_style_text_color(hdgUnitDeg,BLACK,0);
    lv_obj_set_style_text_font(hdgUnitDeg, &lv_font_montserrat_12, 0);

  hdgValueObj = lv_label_create(canvas);
    lv_label_set_text_fmt(hdgValueObj,"%d",hdgValue);
    lv_obj_set_pos(hdgValueObj,211,393);
    lv_obj_set_style_text_color(hdgValueObj,BLACK,0);
    lv_obj_set_style_text_font(hdgValueObj, &lv_font_montserrat_24, 0);
}


lv_obj_t *qnhObj;
lv_obj_t *qnhUnitObj;
lv_obj_t *qnhValueObj;
int16_t qnhValue = 1013;
void createQNH(){
  qnhObj = lv_obj_create(canvas);
  lv_obj_remove_style_all(qnhObj);
  lv_obj_set_size(qnhObj,95,30);
  lv_obj_set_pos(qnhObj,349,373);
  lv_obj_set_style_bg_opa(qnhObj,LV_OPA_COVER,0);
  lv_obj_set_style_bg_color(qnhObj,BLACK,0);
    qnhUnitObj = lv_label_create(qnhObj);
    lv_label_set_text(qnhUnitObj,"hPa");
    lv_obj_set_pos(qnhUnitObj,48,9);
    lv_obj_set_style_text_color(qnhUnitObj,CYAN,0);
    lv_obj_set_style_text_font(qnhUnitObj, &lv_font_montserrat_12, 0);

    qnhValueObj = lv_label_create(qnhObj);
    lv_label_set_text_fmt(qnhValueObj,"%d",qnhValue);
    lv_obj_set_pos(qnhValueObj,7,5);
    lv_obj_set_style_text_color(qnhValueObj,CYAN,0);
    lv_obj_set_style_text_font(qnhValueObj, &lv_font_montserrat_20, 0);
}


lv_obj_t *iasObj;
lv_obj_t *iasUnitObj;
lv_obj_t *iasValueObj;
int16_t iasValue = 888;
void createIAS(){
  iasObj = lv_obj_create(canvas);
  lv_obj_remove_style_all(iasObj);
  lv_obj_set_size(iasObj,100,60);
  lv_obj_set_pos(iasObj,0,210);
  
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.radius = 0;
    rect_dsc.bg_opa = LV_OPA_COVER;
    rect_dsc.bg_color = BLACK;

    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(90, 60)];
    lv_obj_t * triangle = lv_canvas_create(iasObj);
    lv_canvas_set_buffer(triangle, cbuf, 90, 60, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_set_pos(triangle,0,0);
    lv_canvas_draw_rect(triangle,0,0,77,60,&rect_dsc);
    const lv_point_t point[]={
      {77,15},
      {90,30},
      {77,45}
    };
    lv_canvas_draw_polygon(triangle,point,3,&rect_dsc);

    iasUnitObj = lv_label_create(iasObj);
    lv_label_set_text(iasUnitObj,"Kmh");
    lv_obj_set_pos(iasUnitObj,37,44);
    lv_obj_set_style_text_color(iasUnitObj,GREYTEXT,0);
    lv_obj_set_style_text_font(iasUnitObj, &lv_font_montserrat_12, 0);

    iasValueObj = lv_label_create(iasObj);
    lv_label_set_text_fmt(iasValueObj,"%d",iasValue);
    lv_obj_set_align(iasValueObj,LV_ALIGN_LEFT_MID);
    lv_obj_set_style_pad_left(iasValueObj,9,0);
    lv_obj_set_style_text_color(iasValueObj,WHITE,0);
    lv_obj_set_style_text_font(iasValueObj, &lv_font_montserrat_30, 0);




}



lv_obj_t *centerLine;
void createCenterLine(){

    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(160, 24)];
    centerLine = lv_canvas_create(canvas);
    lv_canvas_set_buffer(centerLine, cbuf, 160, 24, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_set_pos(centerLine,160,236);

    static lv_point_t line_points[] = { {0, 4}, {60, 4}, {80, 24}, {100, 4}, {160, 4} };

    lv_draw_line_dsc_t lineDesc;
    lv_draw_line_dsc_init(&lineDesc);
    lineDesc.width = 4;
    lineDesc.color = MAGENTA;


    lv_draw_arc_dsc_t arcDesc;
    lv_draw_arc_dsc_init(&arcDesc);
    arcDesc.width = 4;
    arcDesc.color = MAGENTA;

    lv_canvas_draw_line(centerLine,line_points, 5, &lineDesc);
    lv_canvas_draw_arc(centerLine,80,4,4,0,360,&arcDesc);

}

lv_obj_t *vsObj;
lv_obj_t *vsUnitObj;
lv_obj_t *vsValueObj;
int vsValue = 1500;
void createVerticalSpeed(){

  vsObj = lv_obj_create(canvas);
  lv_obj_remove_style_all(vsObj);
  lv_obj_set_size(vsObj,90,80);
  lv_obj_set_pos(vsObj,348,20);  
  lv_obj_set_style_bg_opa(vsObj,LV_OPA_COVER,0);
  lv_obj_set_style_bg_color(vsObj,BLACK,0);

    vsUnitObj = lv_label_create(vsObj);
    lv_label_set_text(vsUnitObj,"ft/m");
    lv_obj_set_pos(vsUnitObj,7,37);
    lv_obj_set_style_text_color(vsUnitObj,WHITE,0);
    lv_obj_set_style_text_font(vsUnitObj, &lv_font_montserrat_12, 0);

    vsValueObj = lv_label_create(vsObj);
    lv_label_set_text_fmt(vsValueObj,"%d",vsValue);
    lv_obj_set_pos(vsValueObj,7,53);
    lv_obj_set_style_text_color(vsValueObj,WHITE,0);
    lv_obj_set_style_text_font(vsValueObj, &lv_font_montserrat_20, 0);
}



lv_obj_t *gsObj;
lv_obj_t *gsUnitObj;
lv_obj_t *gsValueObj;
int gsValue = 888;
void createGS(){
  gsObj = lv_obj_create(canvas);
  lv_obj_remove_style_all(gsObj);
  lv_obj_set_size(gsObj,95,30);
  lv_obj_set_pos(gsObj,36,373);
  lv_obj_set_style_bg_opa(gsObj,LV_OPA_COVER,0);
  lv_obj_set_style_bg_color(gsObj,BLACK,0);
    gsUnitObj = lv_label_create(gsObj);
    lv_label_set_text(gsUnitObj,"GS");
    lv_obj_set_pos(gsUnitObj,21,7);
    lv_obj_set_style_text_color(gsUnitObj,MAGENTA,0);
    lv_obj_set_style_text_font(gsUnitObj, &lv_font_montserrat_12, 0);

    gsValueObj = lv_label_create(gsObj);
    lv_label_set_text_fmt(gsValueObj,"%d",gsValue);
    lv_obj_set_pos(gsValueObj,42,5);
    lv_obj_set_style_text_color(gsValueObj,MAGENTA,0);
    lv_obj_set_style_text_font(gsValueObj, &lv_font_montserrat_20, 0);
}


lv_obj_t *altitudeObj;
lv_obj_t *altitudeUnitObj;
lv_obj_t *altitudeValueObj;
int altitudeValue = 888;
void createAltitude(){
}



void createIASGauge(){
  static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_ALPHA_2BIT(100,364)];
  lv_obj_t *iasGaugeCanvas = lv_canvas_create(canvas);
  lv_canvas_set_buffer(iasGaugeCanvas, cbuf, 100,364, LV_IMG_CF_ALPHA_2BIT);
  lv_obj_set_pos(iasGaugeCanvas,0,58);
  drawArc(iasGaugeCanvas,false);

}
void createAltGauge(){
  static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_ALPHA_2BIT(100,364)];
  lv_obj_t *altGaugeCanvas = lv_canvas_create(canvas);
  lv_canvas_set_buffer(altGaugeCanvas, cbuf, 100,364, LV_IMG_CF_ALPHA_2BIT);
  lv_obj_set_pos(altGaugeCanvas,380,58);
  drawArc(altGaugeCanvas,true);
}
void drawArc(lv_obj_t *canvas,bool inverted) {
    int w = 18;
    int h = 364;
    float angle = 2*atan((h/2)/w);
    float r = h / (2*sin(angle));
    float trimmedR = r - w;

    for (int x = 0; x < 100; x++) {
        for (int y = 0; y < h; y++) {
          float refW = inverted ? trimmedR + w - x : trimmedR + x - 82;
          float refH = (h/2)-y;
          float refR = sqrt(pow(refW,2)+pow(refH,2));
          if((!inverted && x<=82) || (inverted && x>w) || refR <= r){
            lv_canvas_set_px_color(canvas, x, y, BLACK);
            lv_canvas_set_px_opa(canvas, x, y, LV_OPA_30);
          }
        }
    }
}


void setup() {
  preSetup();

  // lv_demo_benchmark();
  createCanvas();

  createHorizon();

  createIASGauge();
  createAltGauge();

  createHDG();

  createVerticalSpeed();

  createQNH();


  createIAS();

  createGS();

  createCenterLine();
}

short int step = 0;
short int direction = 10;
const int bottomLimit = -30;
const int topLimit = 30;
short int lastmillis = 0;
const int maxmillis = 33;
const int second = 100;
TaskHandle_t Task1;

void loop() {
  if(chipTarget) doLoop();
}
void doLoop(){
  const long actual = millis();
  const int deltaTime = actual-lastmillis;
  if(deltaTime>maxmillis){
    updateHorizon();
    lastmillis = millis();
    switch (step){
      case 0:
        roll+=direction;
        if(roll > 20) step++;
        break;
      case 1:
        pitch+=direction*2;
        if(pitch > 60) step++;
        break;
      case 2:
        roll-=direction;
        if(roll < -20) step++;
        break;
      case 3:
        pitch-=direction;
        if(pitch < -30) step=0;
        break;
    };
  }else{
    lv_timer_handler();
  }
  // chipTarget = !chipTarget;
}
void lcd_send_data(uint8_t data) {
  uint8_t n;
  for (n = 0; n < 8; n++) {
    if (data & 0x80)
      xl.digitalWrite(LCD_SDA_PIN, 1);
    else
      xl.digitalWrite(LCD_SDA_PIN, 0);

    data <<= 1;
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
  }
}

void lcd_cmd(const uint8_t cmd) {
  xl.digitalWrite(LCD_CS_PIN, 0);
  xl.digitalWrite(LCD_SDA_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 1);
  lcd_send_data(cmd);
  xl.digitalWrite(LCD_CS_PIN, 1);
}

void lcd_data(const uint8_t *data, int len) {
  uint32_t i = 0;
  if (len == 0)
    return; // no need to send anything
  do {
    xl.digitalWrite(LCD_CS_PIN, 0);
    xl.digitalWrite(LCD_SDA_PIN, 1);
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(*(data + i));
    xl.digitalWrite(LCD_CS_PIN, 1);
    i++;
  } while (len--);
}

void tft_init(void) {
  xl.digitalWrite(LCD_CS_PIN, 1);
  xl.digitalWrite(LCD_SDA_PIN, 1);
  xl.digitalWrite(LCD_CLK_PIN, 1);

  // Reset the display
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  int8_t cmd = 0;
  while (st_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    cmd++;
  }
}
