#ifndef JPEG_MCU_BLOCK_H
#define JPEG_MCU_BLOCK_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "jpeg_bit_buffer.h"
#include "jpeg_dht.h"

#define dprintf     
//定义一个空操作
//-----------------------------------------------------------------------------
// jpeg_mcu_block: c++中，class是一个类，jpeg_mcu_block就是一个类的名称
//                 一个类定义了一组特定的数据和操作这些数据的方法。
//-----------------------------------------------------------------------------
class jpeg_mcu_block
{
public: // literally，公有的,public关键字定义的成员变量和成员函数可以被任何对象或类访问。
    jpeg_mcu_block(jpeg_bit_buffer *bit_buf, jpeg_dht *dht) //对象的创建 构造函数初始化类的实例
    {
        m_bit_buffer = bit_buf;  //这是一条赋值语句，将参数bit_buf的值幅给了成员变量m_bit_buffer，
                                 //而m_bit_buffer是jpeg_mcu_block类的一个私有成员变量
        m_dht        = dht;
        reset();                //这是一条函数调用语句，调用了jpeg_mcu_block类中定义的reset方法。
    }

    void reset(void) { }

    //-----------------------------------------------------------------------------
    // decode: Run huffman entropy decoder on input stream, expand to DC + upto 
    //         63 AC samples.  64 = 1 DC + 63 AC 样本
    //-----------------------------------------------------------------------------
    int decode(int table_idx, int16_t &olddccoeff, int32_t *block_out)
    {
        int samples = 0;

        
        for (int coeff=0;coeff<64;coeff++)
        {
            // Read 32-bit word
            uint32_t input_word = m_bit_buffer->read_word();

            // Start with upper 16-bits
            uint16_t input_data = input_word >> 16;

            // Perform huffman decode on input data (code=RLE,num_bits)
            uint8_t code   = 0;
            int code_width = m_dht->lookup(table_idx + (coeff != 0), input_data, code); //使用哈夫曼查找表进行解码
            int coef_bits  = code & 0xF;    //低四位，DC系数表示DC系数的系数需要的比特数，AC系数则低四位表示系数需要的比特数，DC系数无高四位


            // Move input point past decoded data     //根据解码的搭配的代码和系数的比特数来推进缓冲区的位置
            if (coeff == 0)
                m_bit_buffer->advance(code_width + coef_bits);
            // End of block or ZRL (no coefficient)
            else if (code == 0 || code == 0xF0)
                m_bit_buffer->advance(code_width);
            else
                m_bit_buffer->advance(code_width + coef_bits);

            // Use remaining data for actual coeffecient   //根据剩余的输入数据来获取实际的系数
            input_data = input_word >> (16 - code_width);   //码字的真实数据

            // DC
            if (coeff == 0)
            {
                input_data >>= (16 - code);

                int16_t dcoeff = decode_number(input_data, coef_bits) + olddccoeff;
                //printf("DC系数：%d\n", dcoeff);
                olddccoeff = dcoeff;                        //DC = 差值 + 前一个DC值
                block_out[samples++] = (0 << 16) | (dcoeff & 0xFFFF);
                
            }                                                 //高16位存系数位置，低16位存非零系数值
            // AC
            else
            {
                // End of block
                if (code == 0)
                {
                    dprintf("SMPL: EOB\n");
                    coeff = 64;
                    break;
                }

                // The first part of the AC key_len is the number of leading zeros
                if (code == 0xF0)  //连续15个零
                {
                    // When the ZRL code comes, it is regarded as 15 zero data
                    dprintf("SMPL: ZRL\n");
                    coeff += 15; // +1 in the loop
                    continue;
                }
                else if (code > 15)
                    coeff   += code >> 4;   //Code的高四位，该非零AC系数前连续的0个数

                input_data >>= (16 - coef_bits);   //AC系数coef_bits位的真实比特串

                if (coeff < 64)
                {
                    int16_t acoeff = decode_number(input_data, coef_bits);
                    //printf("AC系数：%d\n", acoeff);
                    block_out[samples++] = (coeff << 16) | (acoeff & 0xFFFF);
                                    //高16位存系数位置，低16位存非零系数值
                }
            }
        }

        return samples;
    }


private:  //contrast to public,private定义的成员变量和成员函数只能被该类的函数访问，不能被类的外部访问。
    //-----------------------------------------------------------------------------
    // decode_number: Extract number from code / width 
    //-----------------------------------------------------------------------------
    int16_t decode_number(uint16_t code, int bits)
    {   
        if (!(code & (1<<(bits-1))) && bits != 0)
        {
            code |= (~0) << bits;
            code += 1;
        }
        return code;
    }

private:
    jpeg_bit_buffer *m_bit_buffer;
    jpeg_dht *m_dht;

};

#endif
