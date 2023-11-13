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
                        //�� #define dprintf�������Ķ���ͨ�����ڵ���Ŀ�ģ�������Ҫ���������Ϣʱ�����Խ��궨��Ϊ printf������ dprintf �����˺� printf һ���Ĺ��ܡ�
                       // debug_printf
//-----------------------------------------------------------------------------
// jpeg_dqt:
//-----------------------------------------------------------------------------
class jpeg_dht      //jpeg_dht���࣬���������ڴ���JPEGͼ���ļ��е���ɢ���ұ任��DHT���Ĳ��֡���JPEG�����У�DHT���ְ�����ͼ�����ݵĹ�����������Ϣ
{
public:
    jpeg_dht() { reset(); }     //����jpeg_dht��Ĺ��캯������������Ķ��󱻴���ʱ�Զ����á���������캯���У���������reset��������ʼ�����һЩ��Ա������

    void reset(void)
    {
        for (int i=0;i<4;i++)
            memset(&m_dht_table[i], 0, sizeof(t_huffman_table));  //��dht����Ϊ��
    }

    int process(uint8_t *data, int len)//����������ڴ������DHT����Ϣ�����ݡ�������һ��ָ�����ݵ�ָ������ݵĳ��ȣ�Ȼ�������Щ���������m_dht_table���顣�����ķ���ֵ�Ǵ�����ֽ�����
    {
        uint8_t *buf = data;
        int consumed = 0;

        // DHT tables can be combined into one section (it seems)
        while (consumed <= (len-17))   //lenӦ����dht���λ���ȣ���Ҫ��ȥ��ͷ��ʶ��2���ֽڣ�Ҳ����-16λ
        {
            // Huffman table info, first four MSBs represent table type (0 for DC, 1 for AC), last four LSBs represent table #
            uint8_t  table_info  = *buf++;

            int table_idx = 0;
            switch (table_info)
            {
                case DHT_TABLE_Y_DC:    //0x00  DC��, ID0
                    table_idx = DHT_TABLE_Y_DC_IDX;
                    break;
                case DHT_TABLE_Y_AC:    //0x10  AC��, ID0
                    table_idx = DHT_TABLE_Y_AC_IDX;
                    break;
                case DHT_TABLE_CX_DC:   //0x01  DC��, ID1
                    table_idx = DHT_TABLE_CX_DC_IDX;
                    break;
                case DHT_TABLE_CX_AC:   //0x11  AC��, ID1
                    table_idx = DHT_TABLE_CX_AC_IDX;
                    break;
                default:
                    assert(!"ERROR: Bad JPEG"); //����
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
                for (int j=0;j<symb_count[x];j++)   //EVERY CODE LENGTH, symb_count ��
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

    // lookup: Perform huffman lookup (starting from bit 15 of w) //���ұ����ڸ�����2�ֽ��������������ȳ��ֵ�����
    int lookup(int table_idx, uint16_t w, uint8_t &value) 
    {
        for (int i=0;i<m_dht_table[table_idx].entries;i++)  //��width��С�����������ϵ����������ƥ������ĸ�
        {
            int      width   = m_dht_table[table_idx].code_len[i];
            uint16_t bitmap  = m_dht_table[table_idx].code[i];
            
            
            uint16_t shift_val = w >> (16-width);       //��������15bit Ҳ��16��Code_length�Ĺ��������ұ�
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
    } t_huffman_table;      //���ڴ洢�����������Ϣ���������롢����ĳ��ȡ���Ӧ��ֵ���Լ����е���Ŀ������

    t_huffman_table m_dht_table[4];
};

#endif
