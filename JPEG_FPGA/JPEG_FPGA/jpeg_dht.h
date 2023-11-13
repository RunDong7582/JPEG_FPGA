#ifndef JPEG_DHT_H
#define JPEG_DHT_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#define DHT_TABLE_Y_DC      0x00
#define DHT_TABLE_Y_DC_IDX  0
#define DHT_TABLE_Y_AC      0x10
#define DHT_TABLE_Y_AC_IDX  1
#define DHT_TABLE_CX_DC     0x01
#define DHT_TABLE_CX_DC_IDX 2
#define DHT_TABLE_CX_AC     0x11
#define DHT_TABLE_CX_AC_IDX 3

#define dprintf  
                        //即 #define dprintf。这样的定义通常用于调试目的，当你需要输出调试信息时，可以将宏定义为 printf，这样 dprintf 就有了和 printf 一样的功能。
                       // debug_printf
//-----------------------------------------------------------------------------
// jpeg_dqt:
//-----------------------------------------------------------------------------
class jpeg_dht      //jpeg_dht的类，该类是用于处理JPEG图像文件中的离散余弦变换（DHT）的部分。在JPEG编码中，DHT部分包含了图像数据的哈夫曼编码信息
{
public:
    jpeg_dht() { reset(); }     //这是jpeg_dht类的构造函数，它会在类的对象被创建时自动调用。在这个构造函数中，它调用了reset函数来初始化类的一些成员变量。

    void reset(void)
    {
        for (int i=0;i<4;i++)
            memset(&m_dht_table[i], 0, sizeof(t_huffman_table));  //将dht表置为零
    }

    int process(uint8_t *data, int len)//这个函数用于处理包含DHT表信息的数据。它接收一个指向数据的指针和数据的长度，然后解析这些数据以填充m_dht_table数组。函数的返回值是处理的字节数。
    {
        uint8_t *buf = data;
        int consumed = 0;

        // DHT tables can be combined into one section (it seems)
        while (consumed <= (len-17))   //len应该是dht表的位长度，需要减去表头标识的2个字节，也就是-16位
        {
            // Huffman table info, first four MSBs represent table type (0 for DC, 1 for AC), last four LSBs represent table #
            uint8_t  table_info  = *buf++;

            int table_idx = 0;
            switch (table_info)
            {
                case DHT_TABLE_Y_DC:    //0x00  DC表, ID0
                    table_idx = DHT_TABLE_Y_DC_IDX;
                    break;
                case DHT_TABLE_Y_AC:    //0x10  AC表, ID0
                    table_idx = DHT_TABLE_Y_AC_IDX;
                    break;
                case DHT_TABLE_CX_DC:   //0x01  DC表, ID1
                    table_idx = DHT_TABLE_CX_DC_IDX;
                    break;
                case DHT_TABLE_CX_AC:   //0x11  AC表, ID1
                    table_idx = DHT_TABLE_CX_AC_IDX;
                    break;
                default:
                    assert(!"ERROR: Bad JPEG"); //断言
                    break;
            }
            dprintf("DHT (Table idx %d)\n", table_idx);

            // Reset table
            memset(&m_dht_table[table_idx], 0, sizeof(m_dht_table[0]));

            // Extract symbol count
            uint8_t symb_count[16];
            for (int x=0;x<16;x++)
            {
                symb_count[x] = *buf++;
                dprintf(" bit length: %d, symbols: %d\n", x, symb_count[x]);
            }

            // Extract table values
            // Build the Huffman map of (length, code) -> value
            uint16_t code = 0;
            int entry = 0;
            for (int x=0;x<16;x++)
            {
                for (int j=0;j<symb_count[x];j++)   //EVERY CODE LENGTH, symb_count 个
                {
                    uint8_t dht_val = *buf++;   //now the pointer buf have point to the start of the symbol number!
                    m_dht_table[table_idx].code[entry]     = code;
                    m_dht_table[table_idx].code_len[entry] = x+1;       //(0~15times represent (x+1) Code length every time)
                    m_dht_table[table_idx].value[entry++]  = dht_val;   //the last updated,so entry plus ++;
                    dprintf(" %d: %x -> %x\n", entry, code, dht_val);

                    code++;
                }
                code <<= 1; //cj = (cj-k + 1) <<k ,cause when it travel end,code in for loop has plused one.
            }
            m_dht_table[table_idx].entries = entry;

            consumed = buf - data; // updated the total processed bytes 
        }

        return buf - data;
    }

    // lookup: Perform huffman lookup (starting from bit 15 of w) //查找表，对于给定的2字节码流，查找最先出现的码字
    int lookup(int table_idx, uint16_t w, uint8_t &value) 
    {
        for (int i=0;i<m_dht_table[table_idx].entries;i++)  //按width从小到达遍历所有系数，看最先匹配的是哪个
        {
            int      width   = m_dht_table[table_idx].code_len[i];
            uint16_t bitmap  = m_dht_table[table_idx].code[i];
            
            
            uint16_t shift_val = w >> (16-width);       //构建的是15bit 也即16个Code_length的哈夫曼查找表，
            //printf("- %d: check against %04x ", width, shift_val);
            //print_bin(shift_val, width);
            //printf(" == %04x -> %02x\n", bitmap, value);
            if (shift_val == bitmap)
            {
                value   = m_dht_table[table_idx].value[i];
                return width;
            }
        }
        return 0;
    }

private:
    typedef struct
    {
        // 16-bit (max) code
        uint16_t code[255];
        // Code length
        uint8_t  code_len[255];
        // Value to translate to
        uint8_t  value[255];
        int      entries;
    } t_huffman_table;      //用于存储哈夫曼表的信息，包括代码、代码的长度、对应的值，以及表中的条目数量。

    t_huffman_table m_dht_table[4];
};

#endif
