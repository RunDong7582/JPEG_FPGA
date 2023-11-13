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
//����һ���ղ���
//-----------------------------------------------------------------------------
// jpeg_mcu_block: c++�У�class��һ���࣬jpeg_mcu_block����һ���������
//                 һ���ඨ����һ���ض������ݺͲ�����Щ���ݵķ�����
//-----------------------------------------------------------------------------
class jpeg_mcu_block
{
public: // literally�����е�,public�ؼ��ֶ���ĳ�Ա�����ͳ�Ա�������Ա��κζ��������ʡ�
    jpeg_mcu_block(jpeg_bit_buffer *bit_buf, jpeg_dht *dht) //����Ĵ��� ���캯����ʼ�����ʵ��
    {
        m_bit_buffer = bit_buf;  //����һ����ֵ��䣬������bit_buf��ֵ�����˳�Ա����m_bit_buffer��
                                 //��m_bit_buffer��jpeg_mcu_block���һ��˽�г�Ա����
        m_dht        = dht;
        reset();                //����һ������������䣬������jpeg_mcu_block���ж����reset������
    }

    void reset(void) { }

    //-----------------------------------------------------------------------------
    // decode: Run huffman entropy decoder on input stream, expand to DC + upto 
    //         63 AC samples.  64 = 1 DC + 63 AC ����
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
            int code_width = m_dht->lookup(table_idx + (coeff != 0), input_data, code); //ʹ�ù��������ұ���н���
            int coef_bits  = code & 0xF;    //����λ��DCϵ����ʾDCϵ����ϵ����Ҫ�ı�������ACϵ�������λ��ʾϵ����Ҫ�ı�������DCϵ���޸���λ


            // Move input point past decoded data     //���ݽ���Ĵ���Ĵ����ϵ���ı��������ƽ���������λ��
            if (coeff == 0)
                m_bit_buffer->advance(code_width + coef_bits);
            // End of block or ZRL (no coefficient)
            else if (code == 0 || code == 0xF0)
                m_bit_buffer->advance(code_width);
            else
                m_bit_buffer->advance(code_width + coef_bits);

            // Use remaining data for actual coeffecient   //����ʣ���������������ȡʵ�ʵ�ϵ��
            input_data = input_word >> (16 - code_width);   //���ֵ���ʵ����

            // DC
            if (coeff == 0)
            {
                input_data >>= (16 - code);

                int16_t dcoeff = decode_number(input_data, coef_bits) + olddccoeff;
                //printf("DCϵ����%d\n", dcoeff);
                olddccoeff = dcoeff;                        //DC = ��ֵ + ǰһ��DCֵ
                block_out[samples++] = (0 << 16) | (dcoeff & 0xFFFF);
                
            }                                                 //��16λ��ϵ��λ�ã���16λ�����ϵ��ֵ
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
                if (code == 0xF0)  //����15����
                {
                    // When the ZRL code comes, it is regarded as 15 zero data
                    dprintf("SMPL: ZRL\n");
                    coeff += 15; // +1 in the loop
                    continue;
                }
                else if (code > 15)
                    coeff   += code >> 4;   //Code�ĸ���λ���÷���ACϵ��ǰ������0����

                input_data >>= (16 - coef_bits);   //ACϵ��coef_bitsλ����ʵ���ش�

                if (coeff < 64)
                {
                    int16_t acoeff = decode_number(input_data, coef_bits);
                    //printf("ACϵ����%d\n", acoeff);
                    block_out[samples++] = (coeff << 16) | (acoeff & 0xFFFF);
                                    //��16λ��ϵ��λ�ã���16λ�����ϵ��ֵ
                }
            }
        }

        return samples;
    }


private:  //contrast to public,private����ĳ�Ա�����ͳ�Ա����ֻ�ܱ�����ĺ������ʣ����ܱ�����ⲿ���ʡ�
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
