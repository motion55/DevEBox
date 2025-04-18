/*
 * OV7670.c
 * OV7670 DCMI DMA Driver
 * Created on: Aug 5, 2024
 *     Author: K.Rudenko
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"
#include "OV7670.h"
//#include "StateM.h"
#include "dcmi.h"
#include "i2c.h"


/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/

#define OV7670_USE_DMA_CMSIS          0

#define OV7670_SCCB_ADDR              (0x42U)

#define OV7670_RGB565_BYTES           (2U)
#define OV7670_LINES_IN_CHUNK         (1U)
#define OV7670_WIDTH_SIZE_BYTES       (OV7670_WIDTH * OV7670_RGB565_BYTES)
#define OV7670_WIDTH_SIZE_WORDS       (OV7670_WIDTH_SIZE_BYTES / 4U)
#define OV7670_HEIGHT_SIZE_BYTES      (OV7670_HEIGHT * OV7670_RGB565_BYTES)
#define OV7670_HEIGHT_SIZE_WORDS      (OV7670_HEIGHT_SIZE_BYTES / 4U)
#define OV7670_FRAME_SIZE_BYTES       (OV7670_WIDTH * OV7670_HEIGHT * OV7670_RGB565_BYTES)
#define OV7670_FRAME_SIZE_WORDS       (OV7670_FRAME_SIZE_BYTES / 4U)

#if (OV7670_STREAM_MODE != OV7670_STREAM_MODE_BY_FRAME)
//#define DISPLAY_BUFFER_ADDR			  (0xD0000000)
#endif
#define DISPLAY_WIDTH_SIZE_BYTES      (DISPLAY_WIDTH * OV7670_RGB565_BYTES)
#define DISPLAY_WIDTH_SIZE_WORDS      (DISPLAY_WIDTH_SIZE_BYTES / 4U)
#define DISPLAY_HEIGHT_SIZE_BYTES     (DISPLAY_HEIGHT * OV7670_RGB565_BYTES)
#define DISPLAY_HEIGHT_SIZE_WORDS     (DISPLAY_HEIGHT_SIZE_BYTES / 4U)

#ifdef	DISPLAY_BUFFER_ADDR
/* Macro for update address to second half of double-line buffer */
#define OV7670_RESET_BUFFER_ADDR()    (uint32_t)DISPLAY_BUFFER_ADDR
#define OV7670_SWITCH_BUFFER()        (uint32_t)(OV7670.buffer_addr + DISPLAY_WIDTH_SIZE_BYTES)
#else
/* Macro for update address to second half of double-line buffer */
#define OV7670_RESET_BUFFER_ADDR()    (uint32_t)img_buffer
#define OV7670_SWITCH_BUFFER()        ((OV7670.buffer_addr==(uint32_t)img_buffer) ? \
        (OV7670.buffer_addr + (OV7670_BUFFER_SIZE_BYTES)/2U) : ((uint32_t)buffer))
#endif

#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
/* For double stream-line buffer */
//#define DISPLAY_BUFFER_SIZE           (DISPLAY_WIDTH_SIZE_BYTES * OV7670_RGB565_BYTES * OV7670_LINES_IN_CHUNK)
#define OV7670_BUFFER_SIZE_BYTES            (OV7670_WIDTH_SIZE_BYTES * OV7670_HEIGHT * OV7670_LINES_IN_CHUNK)
#define OV7670_DMA_DATA_LEN           (OV7670_WIDTH_SIZE_WORDS * OV7670_LINES_IN_CHUNK)

#if	1
#define OV7670_START_XLK(htim, channel)
#define OV7670_STOP_XLK(htim, channel)
#else
#define OV7670_START_XLK(htim, channel)\
        do{SET_BIT(htim->Instance->CCER, (0x1UL << channel));\
        SET_BIT(htim->Instance->CR1, TIM_CR1_CEN);}while(0)

#define OV7670_STOP_XLK(htim, channel)\
        do{CLEAR_BIT(htim->Instance->CCER, (0x1UL << channel));\
        CLEAR_BIT(htim->Instance->CR1, TIM_CR1_CEN);}while(0)
#endif

#if (OV7670_USE_DMA_CMSIS == 1)
#define OV7670_DCMI_DMA_START(hdcmi, dstAddr)\
    do{\
        WRITE_REG(hdcmi->DMA_Handle->Instance->M0AR, dstAddr);\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_TC_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_HT_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_TE_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_DME_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_FE_FLAG_INDEX(hdcmi->DMA_Handle));\
        SET_BIT(hdcmi->DMA_Handle->Instance->CR, DMA_SxCR_EN);\
        SET_BIT(hdcmi->Instance->CR, DCMI_CR_CAPTURE);\
    }while(0)

#define OV7670_DCMI_DMA_STOP(hdcmi) \
    do{CLEAR_BIT(hdcmi->Instance->CR, DCMI_CR_CAPTURE);\
        CLEAR_BIT(hdcmi->DMA_Handle->Instance->CR, DMA_SxCR_EN);\
        while (READ_BIT(hdcmi->DMA_Handle->Instance->CR, DMA_SxCR_EN));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_TC_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_HT_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_TE_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_DME_FLAG_INDEX(hdcmi->DMA_Handle));\
        __HAL_DMA_CLEAR_FLAG(hdcmi->DMA_Handle, __HAL_DMA_GET_FE_FLAG_INDEX(hdcmi->DMA_Handle));\
    }while(0)
#endif

#else
/* For whole-size snapshot buffer */
#define OV7670_BUFFER_SIZE_BYTES       (OV7670_FRAME_SIZE_BYTES)
#define OV7670_DMA_DATA_LEN            (OV7670_FRAME_SIZE_WORDS)

#define OV7670_START_XLK(htim, channel)
#define OV7670_STOP_XLK(htim, channel)
#endif


const uint8_t OV7670_reg[][2] =
{
  /* Color mode and resolution settings */
  {OV7670_REG_COM7,             0x14},         // QVGA, RGB
//{OV7670_REG_COM7,             0xCU},         // QCIF (176*144), RGB
  {OV7670_REG_RGB444,           0x00},         // RGB444 Disable
  {OV7670_REG_COM15,            0xD0},         // RGB565
  {OV7670_REG_TSLB,             0xCU},         // UYVY
  {OV7670_REG_COM13,            0x80},         // gamma enable, UV auto adjust, UYVY
  {OV7670_REG_RSRVD,            0x84},         // Important!
  /* Clock settings */
  {OV7670_REG_COM3,             0x04},         // DCW enable
  {OV7670_REG_COM14,            0x19},         // manual scaling, pclk/=2
  {OV7670_REG_SCALING_XSC,      0x3A},         // scaling_xsc
  {OV7670_REG_SCALING_YSC,      0x35},         // scaling_ysc
  {OV7670_REG_SCALING_DCWCTR,   0x11},         // down sample by 2
  {OV7670_REG_SCALING_PCLK_DIV, 0xf1},         // DSP clock /= 2
  /* Windowing */
  {OV7670_REG_HSTART,           0x16},         // HSTART
  {OV7670_REG_HSTOP,            0x04},         // HSTOP
  {OV7670_REG_HREF,             0x80},         // HREF
  {OV7670_REG_VSTRT,            0x03},         // VSTART =  14 ( = 3 * 4 + 2)
  {OV7670_REG_VSTOP,            0x7b},         // VSTOP  = 494 ( = 123 * 4 + 2)
  {OV7670_REG_VREF,             0x0a},         // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)
  /* Color matrix coefficient */
#if 0
  {OV7670_REG_MTX1,             0xb3},
  {OV7670_REG_MTX2,             0xb3},
  {OV7670_REG_MTX3,             0x00},
  {OV7670_REG_MTX4,             0x3d},
  {OV7670_REG_MTX5,             0xa7},
  {OV7670_REG_MTX6,             0xe4},
  {OV7670_REG_MTXS,             0x9e},
#else
  {OV7670_REG_MTX1,             0x80},
  {OV7670_REG_MTX2,             0x80},
  {OV7670_REG_MTX3,             0x00},
  {OV7670_REG_MTX4,             0x22},
  {OV7670_REG_MTX5,             0x5E},
  {OV7670_REG_MTX6,             0x80},
  {OV7670_REG_MTXS,             0x9E},
#endif
//{OV7670_REG_COM8,             0x84},
//{OV7670_REG_COM9,             0x0a},         // AGC Ceiling = 2x
//{0x5FU,                       0x2f},         // AWB B Gain Range (empirically decided)
        // without this bright scene becomes yellow (purple). might be because of color matrix
//{0x60U,                       0x98},         // AWB R Gain Range (empirically decided)
//{0x61U,                       0x70},         // AWB G Gain Range (empirically decided)
  {OV7670_REG_COM16,            0x38},         // edge enhancement, de-noise, AWG gain enabled
  /* gamma curve */
#if 1
  {OV7670_REG_GAM1,             16},
  {OV7670_REG_GAM2,             30},
  {OV7670_REG_GAM3,             53},
  {OV7670_REG_GAM4,             90},
  {OV7670_REG_GAM5,             105},
  {OV7670_REG_GAM6,             118},
  {OV7670_REG_GAM7,             130},
  {OV7670_REG_GAM8,             140},
  {OV7670_REG_GAM9,             150},
  {OV7670_REG_GAM10,            160},
  {OV7670_REG_GAM11,            180},
  {OV7670_REG_GAM12,            195},
  {OV7670_REG_GAM13,            215},
  {OV7670_REG_GAM14,            230},
  {OV7670_REG_GAM15,            244},
  {OV7670_REG_SLOP,             16},
#else
  /* gamma = 1 */
  {OV7670_REG_GAM1,             4},
  {OV7670_REG_GAM2,             8},
  {OV7670_REG_GAM3,             16},
  {OV7670_REG_GAM4,             32},
  {OV7670_REG_GAM5,             40},
  {OV7670_REG_GAM6,             48},
  {OV7670_REG_GAM7,             56},
  {OV7670_REG_GAM8,             64},
  {OV7670_REG_GAM9,             72},
  {OV7670_REG_GAM10,            80},
  {OV7670_REG_GAM11,            96},
  {OV7670_REG_GAM12,            112},
  {OV7670_REG_GAM13,            144},
  {OV7670_REG_GAM14,            176},
  {OV7670_REG_GAM15,            208},
  {OV7670_REG_SLOP,             64},
#endif
  /* FPS */
//{OV7670_REG_DBLV,             0x4a},         // PLL  x4
  {OV7670_REG_CLKRC,            0x00},         // Pre-scalar = 1/1
  /* Others */
  {OV7670_REG_MVFP,             0x31},         // Mirror flip
//{OV7670_REG_COM17,            0x08},         // Test screen with color bars
  {OV7670_REG_DUMMY,            OV7670_REG_DUMMY},
};
/******************************************************************************
 *                           LOCAL DATA TYPES                                 *
 ******************************************************************************/

 /* Draw line callback type */
 typedef void (*drawLine_cb_t)(const uint8_t *buffer, uint32_t buf_size, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

 /* Draw frame callback type */
 typedef void (*drawFrame_cb_t)(const uint8_t *buffer, uint32_t buf_size);

enum
{
    FALSE, TRUE
};

enum
{
    BUSY, READY
};

/******************************************************************************
 *                         LOCAL DATA PROTOTYPES                              *
 ******************************************************************************/

static struct
{
    /* HAL peripheral handlers */
    DCMI_HandleTypeDef  *hdcmi;
    I2C_HandleTypeDef   *hi2c;
    TIM_HandleTypeDef   *htim;
    uint32_t            tim_ch;
    /* Requested mode */
    uint32_t            mode;
    /* Address of the buffer */
    volatile uint32_t   buffer_addr;
    /* Image line counter */
    volatile uint32_t   lineCnt;
    /* Driver status */
    volatile uint8_t    state;
    /* Draw line callback prototype */
    drawLine_cb_t       drawLine_cb;
    /* Draw frame callback prototype */
    drawFrame_cb_t      drawFrame_cb;
} OV7670 = {
		.hdcmi = &hdcmi,
		.hi2c = &hi2c_dcmi,
		.htim = 0,
};

/* Image buffer */
#ifdef	DISPLAY_BUFFER_ADDR
uint8_t* img_buffer = (uint8_t*)DISPLAY_BUFFER_ADDR;
#else
uint8_t img_buffer[OV7670_BUFFER_SIZE_BYTES] CAM_BUFFER_SECTION;
#endif

uint32_t line_counter;
uint32_t frame_counter;
uint32_t DCMI_IER;



/******************************************************************************
 *                       LOCAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data);
static HAL_StatusTypeDef SCCB_Read(uint8_t regAddr, uint8_t *data);
static uint8_t isFrameCaptured(void);

/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

void OV7670_Init(DCMI_HandleTypeDef *hdcmi, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim, uint32_t tim_ch)
{
    OV7670.hdcmi = hdcmi;
    OV7670.hi2c = hi2c;
    OV7670.htim = htim;
    OV7670.tim_ch = tim_ch;

    /* PWDN to LOW */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_PWDN, OV7670_GPIO_PIN_PWDN, GPIO_PIN_RESET);
    /* RET pin to LOW */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_RET, OV7670_GPIO_PIN_RET, GPIO_PIN_RESET);
    OV7670_DELAY(100);
    /* RET pin to HIGH */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_RET, OV7670_GPIO_PIN_RET, GPIO_PIN_SET);
    OV7670_DELAY(100);

    /* Start camera XLK signal to be able to do initialization */
    if (OV7670.htim!=0)
    {
        OV7670_START_XLK(OV7670.htim, OV7670.tim_ch);
    }

    /* Do camera reset */
    SCCB_Write(OV7670_REG_COM7, 0x80);
    OV7670_DELAY(30);

    /* Get camera ID */
    uint8_t buf[4];
    SCCB_Read(OV7670_REG_VER, buf);
    DebugPrint("[OV7670] dev id = 0x%02X", buf[0]);

    /* Do camera reset */
    SCCB_Write(OV7670_REG_COM7, 0x80);
    OV7670_DELAY(30);

    /* Do camera configuration */
    for (uint32_t i = 0; OV7670_reg[i][0] != OV7670_REG_DUMMY; i++)
    {
        SCCB_Write(OV7670_reg[i][0], OV7670_reg[i][1]);
        OV7670_DELAY(1);
    }
    /* Stop camera XLK signal */
    if (OV7670.htim!=0)
    {
        OV7670_STOP_XLK(OV7670.htim, OV7670.tim_ch);
    }

    /* Initialize buffer address */
#ifdef	DISPLAY_BUFFER_ADDR
    OV7670.buffer_addr = (uint32_t) DISPLAY_BUFFER_ADDR;
#else
    OV7670.buffer_addr = (uint32_t) img_buffer;
#endif

#if (OV7670_USE_DMA_CMSIS == 1)
    /**************************************************************
     * DMA post-configuration (moved from HAL_DCMI_Start_DMA() API
     **************************************************************/
    /* Enable DCMI by setting DCMIEN bit */
    SET_BIT(OV7670.hdcmi->Instance->CR, DCMI_CR_ENABLE);
    /* Configure the DCMI Mode */
    CLEAR_BIT(OV7670.hdcmi->Instance->CR, DCMI_CR_CM);
    SET_BIT(OV7670.hdcmi->Instance->CR, DCMI_MODE_CONTINUOUS);
    /* Disable double bufferization DMA mode */
    CLEAR_BIT(OV7670.hdcmi->DMA_Handle->Instance->CR, DMA_SxCR_DBM);
    /* Configure DMA Stream data length */
    WRITE_REG(OV7670.hdcmi->DMA_Handle->Instance->NDTR, OV7670_DMA_DATA_LEN);
    /* Configure DMA Stream source address */
    WRITE_REG(OV7670.hdcmi->DMA_Handle->Instance->PAR, (uint32_t) &hdcmi->Instance->DR);
    /* Configure DMA Stream destination address */
    WRITE_REG(OV7670.hdcmi->DMA_Handle->Instance->M0AR, OV7670.buffer_addr);
    /* Clear all DMA interrupt flags */
    __HAL_DMA_CLEAR_FLAG(OV7670.hdcmi->DMA_Handle, __HAL_DMA_GET_TC_FLAG_INDEX(OV7670.hdcmi->DMA_Handle));
    __HAL_DMA_CLEAR_FLAG(OV7670.hdcmi->DMA_Handle, __HAL_DMA_GET_HT_FLAG_INDEX(OV7670.hdcmi->DMA_Handle));
    __HAL_DMA_CLEAR_FLAG(OV7670.hdcmi->DMA_Handle, __HAL_DMA_GET_TE_FLAG_INDEX(OV7670.hdcmi->DMA_Handle));
    __HAL_DMA_CLEAR_FLAG(OV7670.hdcmi->DMA_Handle, __HAL_DMA_GET_DME_FLAG_INDEX(OV7670.hdcmi->DMA_Handle));
    __HAL_DMA_CLEAR_FLAG(OV7670.hdcmi->DMA_Handle, __HAL_DMA_GET_FE_FLAG_INDEX(OV7670.hdcmi->DMA_Handle));
    /* Enable Common interrupts*/
    SET_BIT(OV7670.hdcmi->DMA_Handle->Instance->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
#endif
}

void OV7670_Start(void)
{
    __disable_irq();
    line_counter = 0;
    /* Update requested mode */
    OV7670.mode = DCMI_MODE_CONTINUOUS;
//#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
    /* Reset buffer address */
    OV7670.buffer_addr = OV7670_RESET_BUFFER_ADDR();
//#endif
    /* Reset line counter */
    OV7670.lineCnt = 0U;
    OV7670.state = BUSY;
    __enable_irq();
    /* Start camera XLK signal to capture the image data */
    if (OV7670.htim!=0)
    {
        OV7670_START_XLK(OV7670.htim, OV7670.tim_ch);
    }
    /* Start DCMI capturing */
#if (OV7670_USE_DMA_CMSIS == 1)
    OV7670_DCMI_DMA_START(OV7670.hdcmi, OV7670.buffer_addr);
#else
    HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS, OV7670.buffer_addr, OV7670_DMA_DATA_LEN);
#endif
}

void OV7670_Stop(void)
{
    while(!isFrameCaptured());
    __disable_irq();
#if (OV7670_USE_DMA_CMSIS == 1)
    OV7670_DCMI_DMA_STOP(OV7670.hdcmi);
#else
    HAL_DCMI_Stop(OV7670.hdcmi);
#endif
    OV7670.state = READY;
    __enable_irq();
    if (OV7670.htim!=0)
    {
    	OV7670_STOP_XLK(OV7670.htim, OV7670.tim_ch);
    }
}

void OV7670_RegisterCallback(OV7670_CB_t cb_type, OV7670_FncPtr_t fnc_ptr)
{
    switch (cb_type)
    {
        case OV7670_DRAWLINE_CBK:
        {
            OV7670.drawLine_cb = (drawLine_cb_t)fnc_ptr;
            break;
        }

        case OV7670_DRAWFRAME_CBK:
        {
            OV7670.drawFrame_cb = (drawFrame_cb_t)fnc_ptr;
            break;
        }

        default:
            break;
    }
}

uint32_t buf_addr = 0x0U;

uint8_t OV7670_isDriverBusy(void)
{
    uint8_t retVal;
    __disable_irq();
    uint32_t _line_counter = line_counter;
    retVal = (OV7670.state == BUSY) ? TRUE : FALSE;
    __enable_irq();
    DebugPrint("\r\n line_counter=%8ld %8ld %8ld %8lX %8lX",
    		_line_counter, frame_counter, OV7670.lineCnt, buf_addr, DCMI_IER);
    return retVal;
}


/******************************************************************************
 *                               HAL CALLBACKS                                *
 ******************************************************************************/



#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_FRAME)

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    /* Disable DCMI Camera interface */
    HAL_DCMI_Stop(OV7670.hdcmi);

    /* Stop camera XLK signal until captured image data is drawn */
    if (OV7670.htim!=0)
    {
    	HAL_TIM_OC_Stop(OV7670.htim, OV7670.tim_ch);
    }

    /* Call Display flush function */
    if (OV7670.drawFrame_cb != NULL)
    {
        OV7670.drawFrame_cb((uint8_t*) OV7670.buffer_addr, OV7670_FRAME_SIZE_BYTES);
    }

    /* Reset line counter */
    OV7670.lineCnt = 0U;
    //TODO: check for full-size QVGA buffer mode
    HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS, OV7670.buffer_addr,
            OV7670_FRAME_SIZE_WORDS);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    frame_counter++;
}

#else

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{

    uint8_t hsync_detected = FALSE;
    uint8_t vsync_detected = FALSE;
    uint8_t state = BUSY;

    DCMI_IER = hdcmi->Instance->IER;

    if ((hdcmi->Instance->SR & DCMI_SR_VSYNC))
    {
        vsync_detected = TRUE;
    }
    else
    if ((hdcmi->Instance->SR & DCMI_SR_HSYNC))
    {
    	hsync_detected = TRUE;
    }

    if (vsync_detected)
    {
        frame_counter++;
    }

    if (hsync_detected)
    {
        line_counter++;
        __disable_irq();
        uint32_t lineCnt = OV7670.lineCnt;
        __enable_irq();

        /* If this line is the last line of the frame */
        if (lineCnt >= OV7670_HEIGHT - 1U)
        {
            /* Disable DCMI Camera interface */
#if (OV7670_USE_DMA_CMSIS == 1)
            OV7670_DCMI_DMA_STOP(OV7670.hdcmi);
#else
            HAL_DCMI_Stop(OV7670.hdcmi);
#endif
            /* Stop camera XLK signal until captured image data is drawn */
            //HAL_TIM_OC_Stop(OV7670.htim, OV7670.tim_ch);
            /* Reset line counter */
            lineCnt = 0U;

            /* Update state if mode is SNAPSHOT */
            if (OV7670.mode == DCMI_MODE_SNAPSHOT)
            {
                state = READY;
                vsync_detected = FALSE;
            }
        }
        else
        {
            /* Increment line counter */
            lineCnt++;
        }

        if (((lineCnt + 1U) % OV7670_LINES_IN_CHUNK) == 0U)
        {
            /* Call Display flush function */
            if (OV7670.drawLine_cb != NULL)
            {
                OV7670.drawLine_cb((uint8_t*) OV7670.buffer_addr, (OV7670_WIDTH_SIZE_BYTES * OV7670_LINES_IN_CHUNK) ,
                        0U, (OV7670_WIDTH - 1U), (lineCnt + 1U - OV7670_LINES_IN_CHUNK), lineCnt);
            }

            /* If driver is still working */
            if (state == BUSY)
            {
                /* Update buffer address with the next half-part */
#ifdef	DISPLAY_BUFFER_ADDR
            	if (lineCnt>0)
            	{
                    buf_addr = OV7670_SWITCH_BUFFER();
            	}
            	else
            	{
                    buf_addr = OV7670_RESET_BUFFER_ADDR();
            	}
#else
            	if (lineCnt&1)
            	{
                    buf_addr = (uint32_t)img_buffer + ((OV7670_BUFFER_SIZE_BYTES)/2U);
            	}
            	else
            	{
                    buf_addr = (uint32_t)img_buffer;
            	}
#endif
                /* Capture next line from the snapshot/stream */
#if (OV7670_USE_DMA_CMSIS == 1)
                OV7670_DCMI_DMA_START(OV7670.hdcmi, buf_addr);
#else
                HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS, buf_addr, OV7670_DMA_DATA_LEN);
#endif
            }
        }

        /* Update line counter */
        __disable_irq();
        OV7670.lineCnt = lineCnt;
        OV7670.state = state;
#ifdef	DISPLAY_BUFFER_ADDR
        OV7670.buffer_addr = buf_addr;
#else
        OV7670.buffer_addr = (buf_addr) ? buf_addr : OV7670.buffer_addr;
#endif
        __enable_irq();
    }
}

#endif /* (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE) */


/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data)
{
    HAL_StatusTypeDef ret = 0xFF;

	for (int try=0; try<3;  try++)
	{
        ret = HAL_I2C_Mem_Write(OV7670.hi2c, OV7670_SCCB_ADDR, regAddr,
                I2C_MEMADD_SIZE_8BIT, &data, 1U, 100U);
        if (ret==HAL_OK) break;
    }
    return ret;
}

static HAL_StatusTypeDef SCCB_Read(uint8_t regAddr, uint8_t *data)
{
    HAL_StatusTypeDef ret = HAL_OK;

	for (int try=0; try<3;  try++)
    {
        /* HAL_I2C_Mem_Read doesn't work because of SCCB protocol(doesn't have ACK) */
        ret = HAL_I2C_Master_Transmit(OV7670.hi2c, OV7670_SCCB_ADDR, &regAddr, 1U, 100U);
        if (ret!=HAL_OK) continue;
        ret = HAL_I2C_Master_Receive(OV7670.hi2c, OV7670_SCCB_ADDR, data, 1U, 100U);
        if (ret==HAL_OK) break;
    }

    return ret;
}

HAL_StatusTypeDef ov7670_write(uint8_t regAddr, uint8_t data)
{
	return SCCB_Write(regAddr, data);
}

HAL_StatusTypeDef ov7670_read(uint8_t regAddr, uint8_t *data)
{
	return SCCB_Read(regAddr, data);
}

static uint8_t isFrameCaptured(void)
{
    uint8_t retVal;
    __disable_irq();
    retVal = (OV7670.lineCnt == 0U) ? TRUE : FALSE;
    __enable_irq();
    return retVal;
}


