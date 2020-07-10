#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <map>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
//#include <rb_msgAndSrv/rb_DoubleBool.h>

//语音指令发布者
ros::Publisher order_pub;
std_msgs::Int16 voice_order;

using namespace std;

//时间处理函数
std::string report_time(int a, int b);


/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		printf(">");
		usleep(150*1000); //防止频繁占用CPU
	}//合成状态synth_status取值请参阅《讯飞语音云API文档》
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

std::string to_string(int val) 
{
    char buf[20];
    sprintf(buf, "%d", val);
    return std::string(buf);
}

void voiceWordsCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;
    int         ret                  = MSP_SUCCESS;
    const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    const char* filename             = "tts_sample.wav"; //合成的语音文件名称

    std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;

    std::string dataString = msg->data;
     
	//语音内容判断
    if(dataString.compare("你是谁") == 0)
    {
        char nameString[100] = "我是华数智能机器人001，很开心为你服务";
        text = nameString;
        std::cout << text << std::endl;
    }
    else if(dataString.compare("上使能") == 0)
    {
		voice_order.data = 0;
		order_pub.publish(voice_order);

        char enableRobotString[50] = "收到";
        text = enableRobotString;
        std::cout << text << std::endl;
    }
	else if(dataString.compare("下使能") == 0)
    {
		voice_order.data = 1;
		order_pub.publish(voice_order);

        char unableRobotString[50] = "收到";
        text = unableRobotString;
        std::cout << text << std::endl;
    }
    else if(dataString.compare("现在几点") == 0)
    {
        //获取当前时间
        struct tm *ptm; 
        long ts; 

        ts = time(NULL); 
        ptm = localtime(&ts); 
		
		std::string string = report_time(ptm-> tm_hour, ptm-> tm_min);
		
        char timeString[string.length() + 1];
		for (int i = 0; i < string.length(); ++i)
			timeString[i] =string[i];

        text = timeString;
        std::cout << text << std::endl;
    }
	else if(dataString.compare("你好") == 0)
    {
		voice_order.data = 2;
		order_pub.publish(voice_order);
		
        char helloString[50] = "你好，很荣幸认识你";
        text = helloString;
        std::cout << text << std::endl;
    }
	else if(dataString.compare("抓娃娃") == 0)
    {
		voice_order.data = 3;
		order_pub.publish(voice_order);

        char toyString[50] = "好的，请稍等";
        text = toyString;
        std::cout << text << std::endl;
    }
	else if(dataString.compare("开启行人检测") == 0)
    {
		voice_order.data = 4;
		order_pub.publish(voice_order);

        char enableVisionString[50] = "好的";
        text = enableVisionString;
        std::cout << text<< std::endl;
    }
	else if(dataString.compare("关闭行人检测") == 0)
    {
		voice_order.data = 5;
		order_pub.publish(voice_order);

        char unableVisionString[50] = "好的";
        text = unableVisionString;
        std::cout << text << std::endl;
    }
	else if(dataString.compare("回原点") == 0)
    {
		voice_order.data = 6;
		order_pub.publish(voice_order);

        char backHomeString[50] = "收到";
        text = backHomeString;
        std::cout << text << std::endl;
    }
	else if(dataString.compare("停止") == 0)
    {
		voice_order.data = 7;
		order_pub.publish(voice_order);

        char stopString[50] = "好的";
        text = stopString;
        std::cout << text << std::endl;
    }
    else
    {
        text = msg->data.c_str();
    }

    /* 文本合成 */
    printf("开始合成 ...\n");
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }
    printf("合成完毕\n");


    unlink("/tmp/cmd");  
    mkfifo("/tmp/cmd", 0777);  
    popen("mplayer -quiet -slave -input file=/tmp/cmd 'tts_sample.wav'","r");
    sleep(3);
}

void toExit()
{
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}



int main(int argc, char* argv[])
{
	int         ret                  = MSP_SUCCESS;
	const char* login_params         = "appid = 5f0535bb, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	* 详细参数说明请参阅《讯飞语音云MSC--API文档》
	*/

	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		/*goto exit ;*///登录失败，退出登录
        toExit();
	}

    ros::init(argc,argv,"TextToSpeech");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("voiceSolve_res", 1000, voiceWordsCallback);
	order_pub = n.advertise<std_msgs::Int16 >("voice_order", 100);

    ros::spin();

// exit:
// 	printf("按任意键退出 ...\n");
// 	getchar();
	MSPLogout(); //退出登录
    ros::waitForShutdown();
	return 0;
}


std::string report_time(int a, int b)
{

	int unitHour = a / 1 % 10;
	int tenHour = a / 10 % 10;
	int unitMinute = b / 1 % 10;
	int tenMinute = b / 10 % 10;

	
	std::string ten[6] = {"零", "十", "二十", "三十", "四十", "五十"};
	std::string unit_first[10] = {"零", "一", "二", "三", "四", "五", "六", "七", "八", "九"};
    std::string unit_end[10] = {"", "一", "二", "三", "四", "五", "六", "七", "八", "九"};
	std::string current_time;
	std::string day_period;

	if ( a <= 6)
		day_period = "凌晨";
	else if (a > 6 && a <= 11)
		day_period = "上午";
	else if (a > 11 && a <= 14)
		day_period = "中午";
	else if (a > 14 && a <= 18)
		day_period = "下午";
	else 
		day_period = "晚上";

	if (a < 10)
		current_time = "现在是北京时间" + day_period + unit_first[a] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else if (a = 10)
		current_time = "现在是北京时间" + day_period + ten[1] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else if (a >= 10 && a < 20)
	    current_time = "现在是北京时间" + day_period + ten[1] + unit_end[unitHour] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else if (a = 20)
		current_time = "现在是北京时间" + day_period + ten[2]  + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else 
		current_time = "现在是北京时间" + day_period + ten[2] + unit_end[unitHour] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	
	return current_time;


}
		
