//============================================================================
// Name        : TestImagePro.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iostream>
#include <fstream>
#include <string>
#include <Magick++.h>
using namespace std;
using namespace Magick;

int Image_Process(string openfile)
{
	bool flag_QRexist=false;
	string tempOpenFile;
	string tempOpenFile2;
	char ch_tempOpenFile2[100];

	tempOpenFile=openfile.substr(0,openfile.length()-4);


	Image original_img;
	//original_img.read("/home/robocup/desktop/1.jpg");
	original_img.read(openfile);
	Pixels original_cache(original_img);
	PixelPacket* original_pixel;
	original_pixel=original_cache.get(0,0,original_img.columns(),original_img.rows());


	Image image;
	//image.read("/home/robocup/desktop/1.jpg");
	image.read(openfile);

	image.modifyImage();
	Pixels image_cache(image);            //像素缓存
	PixelPacket* mypixel;

	uint m,n,red_num,a,b,m1,n1,height,startp_x,startp_y,subimg_h,subimg_w,subimg_eh,subimg_ew,temp_m,temp_n;
	uint step=image.columns();
	PixelPacket* mypixel1;
	PixelPacket* mypixel2;
	bool longline_flag=false;
	long num_blackp;
	uint num_subimg=0;
	char filename[50];

	mypixel=image_cache.get(0,0,image.columns(),image.rows());
	long int size=image.columns()*image.rows();
	for(int i=0;i<size;i++)                                         //图像二值化处理
	{
		//(mypixel+i)->blue=0;
		//(mypixel+i)->green=0;
		//(mypixel+i)->red=0;
		(mypixel+i)->green=(mypixel+i)->blue;
		(mypixel+i)->red=(mypixel+i)->blue;

		if(((mypixel+i)->blue)>3*MaxRGB/5)
		{
			(mypixel+i)->blue=MaxRGB;
			(mypixel+i)->red=MaxRGB;
			(mypixel+i)->green=MaxRGB;
		}
		else
		{
			(mypixel+i)->blue=0;
			(mypixel+i)->red=0;
			(mypixel+i)->green=0;
		}

	}



	for(m=0;m<image.rows();m++)               //找到黑白分界点          //行
	{
		for(n=0;n<image.columns()-1;n++)                             //列
		{
			//pixel_1=image.pixelColor(n,m);
			//pixel_2=image.pixelColor(n+1,m);
			mypixel1=mypixel+m*step+n;
			mypixel2=mypixel+m*step+n+1;
			if((mypixel1->red==0)&&(mypixel2->red==MaxRGB))       //把分界处的黑色像素点转成红色
		    {
				mypixel1->red=MaxRGB;
				mypixel1->green=0;
				mypixel1->blue=0;
		    }


		}

	}

	for(m=0;m<image.rows();m++)                                      //行     根据分界线寻找分割感兴趣区域
	{
			for(n=0;n<image.columns();n++)                             //列
			{
					mypixel1=mypixel+m*step+n;
					m1=0;
					n1=0;
					red_num=0;
					num_blackp=0;
					if((mypixel1->red==MaxRGB)&&(mypixel1->green==0)&&(mypixel1->blue==0))
					{
						red_num=1;
						m1=m;
						n1=n;
						temp_m=m;
						temp_n=n;

						for(a=m+1;a<image.rows();a++)
						{
							//if((temp_n-10)>0)
							//	b=temp_n-10;
							if((temp_n-5)>0)
								b=temp_n-5;
							else if((temp_n-3)>0)
								b=temp_n-3;
							else
								b=temp_n-2;

							//if((temp_n-10)<0)
						//		b=0;

							for(;b<temp_n+5;b++)
							{
								mypixel1=mypixel+a*step+b;
								if((mypixel1->red==MaxRGB)&&(mypixel1->green==0)&&(mypixel1->blue==0))
								{
									red_num++;
									m1=(m1<a)?a:m1;
									n1=(n1>b)?b:n1;
									//mypixel1->blue=MaxRGB;
									//mypixel1->red=MaxRGB;
								    //mypixel1->green=MaxRGB;
									mypixel1->blue=0;
									mypixel1->red=0;
									mypixel1->green=MaxRGB;

									temp_n=b;
								    break;

								}


							}
							if(b==(temp_n+5))
								break;

						}


					}
					if(red_num>50)                            //左边沿长度大于50个像素点
					{
						longline_flag=true;
						height=m1-m;
						//startp_x=(int)(n1-height/4);
						//startp_y=(int)(m-height/4);
						startp_x=n1;                          //原图像上感兴趣区域的原点坐标
						startp_y=m;
						//cout<<m1<<endl<<m<<endl;
						//cout<<startp_x<<endl<<startp_y<<endl<<height<<endl;

						subimg_eh=height+startp_y;            //原图像上感兴趣区域的右下角点坐标
						subimg_ew=height+startp_x;
						if(subimg_eh>image.rows())
						{
							subimg_eh=image.rows();
							subimg_h=image.rows()-startp_y;    //感兴趣区域的高度
						}
						else
						{
							subimg_h=height;
						}

						if(subimg_ew>image.columns())
						{
							subimg_ew=image.columns();
							subimg_w=image.columns()-startp_x;   //感兴趣区域的宽度
						}
						else
						{
							subimg_w=height;
						}



						Image sub_image(Geometry(subimg_w,subimg_h),Color(MaxRGB, MaxRGB, MaxRGB, 0));
						sub_image.modifyImage();
						Pixels subimage_cache(sub_image);            //像素缓存
						PixelPacket* sub_mypixel;                    //指向像素缓存中第一个像素点的指针
						sub_mypixel=subimage_cache.get(0,0,sub_image.columns(),sub_image.rows());
						long ssize=sub_image.columns()*sub_image.rows();      //小图像的大小



						for(uint i=startp_y;i<subimg_eh;i++)
						{
							for(uint j=startp_x;j<subimg_ew;j++)
							{
								if(i<0)
									i=0;
								if(i>image.rows())
									i=image.rows();
								if(j<0)
									j=0;
								if(j>image.columns())
									j=image.columns();

								//*sub_mypixel=*(mypixel+i*step+j);
								*sub_mypixel=*(original_pixel+i*step+j);
								sub_mypixel++;

								//原图像上感兴趣区域的红色像素点全部还原成黑色
								if(((mypixel+i*step+j)->green==0)&&((mypixel+i*step+j)->red==MaxRGB)&&((mypixel+i*step+j)->blue==0))
								{
									(mypixel+i*step+j)->green=0;
									(mypixel+i*step+j)->red=0;
									(mypixel+i*step+j)->blue=0;
								}

								//计算感兴趣区域的黑色像素点个数
								if(((mypixel+i*step+j)->green==0)&&((mypixel+i*step+j)->red==0)&&((mypixel+i*step+j)->blue==0))
								{
									num_blackp++;
								}

							}

						}
						//cout<<num_blackp<<endl<<ssize<<endl;

						if((num_blackp*5) > ssize)   //黑色像素点占图片总大小的五分之一
						{
							flag_QRexist=true;
							num_subimg++;
							subimage_cache.sync();
							//sub_image.display();

							sprintf(ch_tempOpenFile2,"_%d.bmp",num_subimg);
							tempOpenFile2=ch_tempOpenFile2;
							tempOpenFile2=tempOpenFile+tempOpenFile2;

							//sprintf(filename,"/home/robocup/desktop/1%d.jpg",num_subimg);
							//string filename1;
							//filename1=filename;
							//sub_image.write(filename1);
							sub_image.write(tempOpenFile2);
							//sub_image.write("/home/robocup/desktop/11.jpg");
						}

					}

			}
	}
	//Quantum i=mypixel->red;
	//cout<<i;

	image_cache.sync();
	//image.display();
	return num_subimg;
}
