/*
 * GLCD.c
 *
 *  Created on: Nov 18, 2021
 *      Author: bjtho
 */

#include "GLCD.h"
#include "msp.h"

static unsigned char GLCD_horizonal_addressing;
static unsigned char GLCD_cursor_x = 0;
static unsigned char GLCD_cursor_y = 0;

unsigned char GLCD_get_cursor_x() {
    return GLCD_cursor_x;
}

unsigned char GLCD_get_cursor_y() {
    return GLCD_cursor_y;
}


/*******************************
 * DRIVER FUNCTIONS
 *******************************/

void GLCD_init(void) {
    // Configure output pins
    GLCD_SPI_PORT->SEL0 |= GLCD_CLK | GLCD_DIN;      // Configure CLK and Data pins to use the UCB0 module
    GLCD_SPI_PORT->SEL1 &= ~(GLCD_CLK | GLCD_DIN);
    GLCD_CTL_PORT->DIR |= (GLCD_CE | GLCD_RST | GLCD_DC); // set as outputs

    // Configure eUSCI_B0 for SPI
    EUSCI_B0->CTLW0 = 0x1;      // Put UCB0 in reset mode
    EUSCI_B0->CTLW0 = 0x69C1;   // Set protocol params
    EUSCI_B0->BRW = 8;          // 48MHz / 8 = 6MHz baud rate <- Fastest operating speed
    EUSCI_B0->CTLW0 &= ~0x1;    // turn off reset bit to enable UCB0


    // Configure display
    GLCD_CTL_PORT->OUT |= GLCD_CE;              // disable data transfer
    GLCD_CTL_PORT->OUT &= ~GLCD_RST;            // assert reset pin on display
    GLCD_CTL_PORT->OUT |= GLCD_RST;             // deassert reset pin on display

    //// send setup commands to display
    GLCD_command_write(0x21);   // extended mode (H = 1)
    GLCD_command_write(0xB8);   // set Vop
    GLCD_command_write(0x04);   // set temp coefficent
    GLCD_command_write(0x14);   // set bias mode
    GLCD_command_write(0x20);   // leave extended mode (H = 0) and set horizonal addressing
    GLCD_horizonal_addressing = 1;

    GLCD_command_write(0x0C);   // set display to normal operating mode

    // clear display
    GLCD_clear();

    GLCD_setCursor(0,0);
}

void GLCD_clear(void) {
    int i;
    for(i = 0;i < (GLCD_WIDTH * GLCD_HEIGHT / 8);i++) {
        GLCD_data_write(0x00);
    }
}

void GLCD_setCursor(unsigned char x, unsigned char y) {
    // constrain cursor to the screen
    GLCD_cursor_x = (x >= GLCD_WIDTH) ? GLCD_WIDTH - 1 : x;
    GLCD_cursor_y = (y >= GLCD_HEIGHT/8) ? GLCD_HEIGHT/8 - 1 : y;
    GLCD_command_write(0x80 | GLCD_cursor_x); // set column (1 pixel / column)
    GLCD_command_write(0x40 | GLCD_cursor_y); // set bank (a.k.a. row) (8 pixels / bank)
}

void GLCD_command_write(unsigned char data) {
    GLCD_CTL_PORT->OUT &= ~GLCD_DC;             // switch to command mode
    SPI_write(data);            // send data to though SPI
}

void GLCD_data_write(unsigned char data) {
    GLCD_CTL_PORT->OUT |= GLCD_DC;             // switch to data mode
    SPI_write(data);            // send data to though SPI
    if(++GLCD_cursor_x == GLCD_WIDTH) {
        GLCD_cursor_x = 0;
        if(++GLCD_cursor_y == GLCD_HEIGHT/8) {
            GLCD_cursor_y = 0;
        }
    }
}

void SPI_write(unsigned char data) {
    GLCD_CTL_PORT->OUT &= ~GLCD_CE;                 // enable data transfer
    EUSCI_B0->TXBUF = data;         // load data and transmit
    while(EUSCI_B0->STATW & BIT0);  // wait for transmission to be done
    GLCD_CTL_PORT->OUT |= GLCD_CE;                  // disable data transfer
}

/*****************************************
 * Text functions
 *******************************************/

int GLCD_itos(int num, char* buffer, int bufferSize) {
    int len = 0;
    if(num == 0) {
        buffer[len++] = '0';
        buffer[len] = '\0';
        return len;
    }

    char isNegative = 0;
    if(num < 0) {
        isNegative = 1;
        num *= -1;
    }
    while(num > 0 && len < bufferSize - isNegative) {
        buffer[len++] = num % 10 + '0';
        num /= 10;
    }
    if(isNegative)
        buffer[len++] = '-';
    buffer[len] = '\0';

    int i;
    for(i = 0;i < len / 2;i++) {
        char temp = buffer[i];
        buffer[i] = buffer[len - i - 1];
        buffer[len - i - 1] = temp;
    }

    return len;
}

void GLCD_print_char(char c, char option) {
    if(!GLCD_horizonal_addressing)
        GLCD_command_write(0x20);

    struct GLCD_char data = GLCD_font[c];
    int i;
    switch(c) {
    case '\n':   // newline
        GLCD_setCursor(0, GLCD_cursor_y + 1);
        break;
    case '\b':   // backspace
        if(GLCD_cursor_x <= 0) {
            if(GLCD_cursor_y > 0) {
                // move cursor up one line and to the closest character to the edge
                // of the screen.
                GLCD_setCursor((GLCD_WIDTH/GLCD_CHAR_W) * GLCD_CHAR_W, GLCD_cursor_y - 1);
            } else {
                break;
            }
        }
        GLCD_setCursor(GLCD_cursor_x - GLCD_CHAR_W, GLCD_cursor_y);
        GLCD_print_char(' ',0);
        GLCD_setCursor(GLCD_cursor_x - GLCD_CHAR_W, GLCD_cursor_y);
        break;
    case '\t':   // tab
        GLCD_setCursor((GLCD_cursor_x / (GLCD_CHAR_W*GLCD_TAB_W) + 1) * (GLCD_CHAR_W*GLCD_TAB_W), GLCD_cursor_y);
        break;
    case '\0':   // null
        break;
    default:
        for(i = 0;i < GLCD_CHAR_W;i++) {
                GLCD_data_write(((i > 0 && option == 'b') ? data.bytes[i-1] : 0x00) | (option == 'u' ? 0x80 : 0x00) | data.bytes[i]);
        }
        break;
    }
}

void GLCD_print_string(char* string, char option) {
    int i;
#if GLCD_WIDTH % GLCD_CHAR_W != 0
    int offset = 0;
#endif
    for(i = 0; string[i] != '\0'; i++) {
        GLCD_print_char(string[i], option);

// Extra code to align lines of text if the provided
// characters don't fit evenly on the screen. This will
// only compile if necessary
#if GLCD_WIDTH % GLCD_CHAR_W != 0
        if((i - offset + 1) % (GLCD_WIDTH / GLCD_CHAR_W) == 0) {
            GLCD_print_char('\n');
        }
        if(string[i] == '\n') {
            offset = (i - offset + 1) % (GLCD_WIDTH / GLCD_CHAR_W);
        }
#endif

    }
}



/*************************************
 * DRAW IMAGE
 **************/
void GLCD_draw_image(int x, int y, struct GLCD_img * img) {
    if(GLCD_horizonal_addressing)
        GLCD_command_write(0x22); // vertical addressing

    unsigned char img_rows = (img->height - 1) / 8 + 1;

    unsigned char byte_offset;
    unsigned char carryByteMask;
    if(y >= 0) {
        byte_offset = y % 8;
        carryByteMask = ~(0xFF >> byte_offset);
    }else {
        byte_offset = -y % 8;
        carryByteMask = ~(0xFF << byte_offset);
    }
    unsigned char c,r,byte_carry;

    unsigned char byte = 0;

    for(c = 0;c < img->width && c < GLCD_WIDTH;c++) {
        if(y >= 0) {
            GLCD_setCursor(x + c,y/8);
            byte_carry = 0;
            for(r = 0;r < img_rows + (byte_offset > 0) && r < GLCD_HEIGHT/8;r++){
                byte = byte_carry;
                if(r < img_rows)
                    byte |= img->bytes[c * img_rows + r] << byte_offset;
                byte_carry = (img->bytes[c * img_rows + r] & carryByteMask) >> (8 - byte_offset);
                GLCD_data_write(byte);
            }
        }else {
            GLCD_setCursor(x + c,0);
            for(r = 0;r - y/8 < img_rows - (byte_offset > 0) && r < GLCD_HEIGHT/8;r++){
                byte = img->bytes[c * (img_rows) + r - y/8] >> byte_offset;
                if(r - y/8 + 1 < img_rows)
                    byte |= (img->bytes[c * (img_rows) + r - y/8 + 1] & carryByteMask) << (8 - byte_offset);
                GLCD_data_write(byte);
            }
        }
    }
}

/*******************************************
 * Image Processing Functions
 *********************************************/
void GLCD_draw_image_onto(int x, int y, struct GLCD_img * src_img, struct GLCD_img * dest_img) {
    GLCD_process_image_onto(x, y, src_img, dest_img, GLCD_ADD_ONTO);
}

void GLCD_remove_image(int x, int y, struct GLCD_img * src_img, struct GLCD_img * dest_img) {
    GLCD_process_image_onto(x, y, src_img, dest_img, GLCD_REMOVE_FROM);
}

void GLCD_clear_image(struct GLCD_img * img) {
    GLCD_process_image(img, GLCD_ERASE);
}


void GLCD_process_image(struct GLCD_img * src_img, void (*callback) (unsigned char * img_byte)) {
    int r,c;
    int rows = (src_img->height - 1) / 8 + 1;
    for(c = 0;c < src_img->width;c++) {
        for(r = 0;r < rows;r++) {
            callback(&(src_img->bytes[c * rows + r]));
        }
    }
}

void GLCD_process_image_onto(int x, int y, struct GLCD_img * src_img, struct GLCD_img * dest_img,
                             void (*callback) (unsigned char * dest_byte, unsigned char * src_byte)) {
    unsigned char src_rows = (src_img->height - 1) / 8 + 1;
    unsigned char dest_rows = (dest_img->height - 1) / 8 + 1;
    int offset_rows = y / 8;

    unsigned char byte_offset;
    unsigned char carryByteMask;
    if(y >= 0) {
        byte_offset = y % 8;
        carryByteMask = ~(0xFF >> byte_offset);
    }else {
        byte_offset = -y % 8;
        carryByteMask = ~(0xFF << byte_offset);
    }

    unsigned char byte_carry;
    int c,r;

    unsigned char byte = 0;

    for(c = (x < 0) ? 0 : x;
            c < GLCD_WIDTH && c < dest_img->width && c - x < src_img->width;c++) {
        if(y >= 0) {
            byte_carry = 0;
            for(r = offset_rows; r < GLCD_HEIGHT/8 && r < dest_rows && (r - offset_rows) < src_rows + (byte_offset > 0);r++){
                byte = byte_carry;
                if(r - offset_rows < src_rows)
                    byte |= src_img->bytes[src_rows * (c - x) + (r - offset_rows)] << byte_offset;
                byte_carry = (src_img->bytes[src_rows * (c - x) + (r - offset_rows)] & carryByteMask) >> (8 - byte_offset);
                callback(&(dest_img->bytes[dest_rows * c + r]), &byte);
            }
        } else {
            for(r = 0; r < GLCD_HEIGHT/8 && r < dest_rows && (r - offset_rows) < src_rows;r++){
                byte = src_img->bytes[src_rows * (c - x) + (r - offset_rows)] >> byte_offset;
                if(r - offset_rows + 1 < src_rows)
                    byte |= (src_img->bytes[src_rows * (c - x) + (r - offset_rows + 1)] & carryByteMask) << (8 - byte_offset);
                callback(&(dest_img->bytes[dest_rows * c + r]), &byte);
            }
        }

    }
}

struct GLCD_img GLCD_combine_images(struct GLCD_img * img1, struct GLCD_img * img2, unsigned char offsetX, unsigned char offsetY) {

    struct GLCD_img output = {
        .width = ((img1->width >= img2->width + offsetX) ? img1->width : img2->width + offsetX),
        .height = ((img1->height >= img2->height + offsetY) ? img1->height : img2->height + offsetY),
        .bytes = {0x00,}
    };

    int output_rows = ((output.height - 1) / 8 + 1);
    int src_rows = ((img1->height - 1) / 8 + 1);
    int addImg_rows = ((img2->height - 1) / 8 + 1);
    int offset_rows = offsetY/8;

    unsigned char byte_offset = offsetY % 8;

    unsigned char c,r, byte;



    unsigned char carryByteMask = ~(0xFF >> byte_offset);
    unsigned char byte_carry = 0;


    for(c = 0;c < output.width && c < GLCD_WIDTH;c++) {
        byte_carry = 0;
        for(r = 0;r < output_rows && r < GLCD_HEIGHT/8;r++) {
            if(r >= GLCD_HEIGHT/8 || c >= GLCD_WIDTH) continue;

            if(c < img1->width && r < src_rows) {
                output.bytes[output_rows * c + r] |= img1->bytes[src_rows * c + r];
            }
            if(c >= offsetX && c < img2->width + offsetX &&
                    r >= offset_rows && r < addImg_rows + offset_rows + (byte_offset > 0)) {
                byte = byte_carry;
                if(r - offset_rows < addImg_rows)
                    byte |= img2->bytes[addImg_rows * (c - offsetX) + (r - offset_rows)] << byte_offset;
                byte_carry = (img2->bytes[addImg_rows * (c - offsetX) + (r - offset_rows)] & carryByteMask) >> (8 - byte_offset);
                output.bytes[output_rows * c + r] |= byte;
            }
        }
    }
    return output;
}

void GLCD_flatten_images(struct GLCD_img * dest_img, struct GLCD_img * img2) {
    *dest_img = GLCD_combine_images(dest_img,img2,0,0);
}

/*************************************
 * Image Generating Functions
 *************************************/

struct GLCD_img GLCD_gen_text_image(char* string,unsigned char w, unsigned char h) {
    if(w > GLCD_WIDTH) w = GLCD_WIDTH;
    if(h > GLCD_HEIGHT) h = GLCD_HEIGHT;
    // create an empty output image
    struct GLCD_img output = {
        .width = w,
        .height = h,
        .bytes ={0x00}
    };

    int i,j;
    // We need a virtual cursor to keep track of where
    // the end of the generated text is in the image.
    unsigned char vcursor_x = 0;
    unsigned char vcursor_y = 0;

    for(i = 0;string[i] != '\0';++i) {

        if(vcursor_y >= h/8) {
            return output; // stop doing things if it's outside the bounds
        }

        switch(string[i]) {
        case '\n':
            // new line character. Move to the next line.
            if(++vcursor_y >= h/8) {
                return output;
            }
            vcursor_x = 0;
            break;
        case '\b':
            // Backspace character

            // don't do anything if there's nothing to backspace
            if(vcursor_x <= 0) {
                if(vcursor_y > 0) {
                    vcursor_y--;
                    // move cursor_x to the maximum edge and
                    // snap it to char_w increments
                    vcursor_x = (w / GLCD_CHAR_W) * GLCD_CHAR_W;
                } else {
                    break;
                }
            }
            // move the cursor back 1 character.
            vcursor_x -= GLCD_CHAR_W;
            // erase pixels after the backspace
            for(j = 0;j < GLCD_CHAR_W;j++) {
                output.bytes[(vcursor_x + j) * h + vcursor_y] = 0x00;
            }
            break;
        case '\t':
            // Tab, move the cursor to the next nearest tab width
            vcursor_x = (vcursor_x / (GLCD_TAB_W * GLCD_CHAR_W) + 1) * (GLCD_TAB_W * GLCD_CHAR_W);
            // re-adjust the cursor
            if(vcursor_x >= w) {
                vcursor_x = 0;
                if(++vcursor_y >= h/8) {
                    return output;
                }
            }
            break;
        case '\0':
            break;
        default:
            // wrap the cursor
            if(vcursor_x + GLCD_CHAR_W > w) {
                vcursor_x = 0;
                if(++vcursor_y >= h/8) {
                    return output;
                }
            }
            // Write the bytes of the character to the output image, adjusting for horizontal addressing
            // to vertical addressing conversion.
            for(j = 0;j < GLCD_CHAR_W;j++) {
                output.bytes[vcursor_x * (h/8) + vcursor_y] = GLCD_font[string[i]].bytes[j];
                ++vcursor_x;
            }
        }
    }
    return output;
}

struct GLCD_img GLCD_gen_char_image(char c) {
    struct GLCD_img output = {
        .width = GLCD_CHAR_W,
        .height = 8,
        .bytes = {0x00}
    };
    int i;
    for(i = 0;i < GLCD_CHAR_W;i++) {
        output.bytes[i] = GLCD_font[c].bytes[i];
    }
    return output;
}

struct GLCD_img GLCD_gen_rect_image(unsigned char w, unsigned char h, unsigned char is_filled) {
    struct GLCD_img output = {
         .width = (w > GLCD_WIDTH) ? GLCD_WIDTH : w,
         .height = (h > GLCD_HEIGHT) ? GLCD_HEIGHT : h,
         .bytes = {0x00}
    };

    int rows = (h - 1)/8 + 1;
    int img_rows = (output.height - 1)/8 + 1;

    int r,c;
    for(c = 0;c < output.width;c++) {
        for(r = 0;r < img_rows;r++) {
            if(is_filled || c == 0 || c == w - 1) {
                if(r < rows - 1)
                    output.bytes[c * img_rows + r] = 0xFF;
                else
                    output.bytes[c * img_rows + r] = ~(0xFF << ((h - 1) % 8 + 1));
            } else {
                if(r == 0) {
                    output.bytes[c * img_rows + r] = 0x01;
                } else if(r == rows - 1){
                    output.bytes[c * img_rows + r] = 0x01 << (h % 8);
                }

            }
        }
    }
    return output;
}

struct GLCD_img GLCD_gen_circle_image(unsigned char radius, float resolution, unsigned char is_filled) {
    int img_height = radius * 0.8 * 2;
    struct GLCD_img output = {
        .width = (radius * 2 >= GLCD_WIDTH) ? GLCD_WIDTH : radius * 2,
        .height = (img_height >= GLCD_HEIGHT) ? GLCD_HEIGHT : img_height,
        .bytes = {0x00}
    };

    int rows = (output.height - 1) / 8 + 1;

    int i;
    float x = 0;
    float y = radius;
    float pi = 3.14159265358979;

    int finalx, finaly;
    for(i = 0;i <= 2 * pi / resolution;i++) {
        finalx = x + radius;
        finaly = (y * 0.8) + radius * 0.8;

        if(finalx < GLCD_WIDTH && finaly < GLCD_HEIGHT)
            output.bytes[finalx * rows + (finaly/8)] |= 1 << (finaly % 8);

        x += y * resolution;
        y -= x * resolution;
    }
    return output;
}
