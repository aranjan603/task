#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>


#define max 200
#define threshold 50
#define infinity 600000

using namespace cv;
using namespace std;

typedef struct POINT
{
   int x;
   int y;
}point;


double dist[max][max];
int visited[max][max];
point pt[max][max];
int COM[2][max];
Mat binary;
int valr,valg;
int flag=0;

void print(Mat img, String windowname)
 {
   namedWindow(windowname, CV_WINDOW_AUTOSIZE);
   imshow(windowname, img);
 }


void find(Mat img)
 {
   int cr=0,cg=0;
   for (int i=0;i<img.rows && (cr==0 || cg==0);i++)
     {
        for (int j=0;j<img.cols && (cr==0 || cg==0);j++)
           {
              if (cr==0 && img.at<Vec3b>(i, j)[0]<40 && img.at<Vec3b>(i, j)[1] <40 && img.at<Vec3b>(i, j)[2]>200 )
                 {
                    cr++;valr=visited[i][j];
                 }
              if (cg==0 && img.at<Vec3b>(i, j)[0] <20 && img.at<Vec3b>(i, j)[1] >220 && img.at<Vec3b>(i, j)[2]<20 )
                 {
                    cg++;valg=visited[i][j];
                 }
            }
      }
 }


Mat Bin(Mat gray)
 {
   Mat binary1(gray.rows, gray.cols, CV_8UC1, 255);
   for (int i = 0; i < gray.rows; i++)
    {
      for (int j = 0; j < gray.cols; j++)
       {
          if ((int)(gray.at<uchar>(i, j)) > threshold)
                binary1.at<uchar>(i, j) = 255;
          else
                binary1.at<char>(i, j) = 0;
       }
    }
   return binary1;
 }


void initialize()
 {
   for (int i = 0; i < max; i++)
     {
        for (int j = 0; j < max; j++)
	   {
		visited[i][j] = infinity;
                pt[i][j].x=-1;
                pt[i][j].y=-1;
		dist[i][j]= infinity;
		if (i<2)
		    COM[i][j] = 0;
	   }
      }
  }


int isvalid(Mat img, int a, int b)
 {
    if (a < 0 || b < 0 || a >= img.rows || b >= img.cols)
        return 0;
    else
        return 1;
 }


void DFSvisit(int i, int j, int count)
 {
    visited[i][j] = count;
    for (int k = i - 1; k <= i + 1; k++)
       {
           for (int l = j - 1; l <= j + 1; l++)
              {
                  if (isvalid(binary, k, l) == 1)
                     {
                         if ((visited[k][l] == infinity) && ((binary.at<uchar>(k, l)) == 255))
                               DFSvisit(k, l, count);
                     }
              }
       }
 }


int DFSutil()
 {
    int count = 1;
    for (int i = 0; i < binary.rows; i++)
       {
          for (int j = 0; j < binary.cols; j++)
             {
               if ((visited[i][j]==infinity) && ((binary.at<uchar>(i, j)) == 255))
                    {
                       DFSvisit(i, j, count);
                       count++;
                    }
             }
       }
    for (int i = 0; i < binary.rows; i++)
      {
         for (int j = 0; j < binary.cols; j++)
            {
               if (visited[i][j] == infinity)
                      visited[i][j] = 0;
            }
       }
    return count;
 }


void centre(int count)
 {
    int single[max];
    for (int i = 1; i < max; i++)
	   single[i] = 0;
    for (int i = 0; i < binary.rows; i++)
	{
	    for (int j = 0; j < binary.cols; j++)
		 {
		    if (visited[i][j] != 0)
			{
				int t = visited[i][j];
				COM[0][t] += i;
				COM[1][t] += j;
				single[t]++;
			}
		 }
	}
    for (int i = 0; i < 2; i++)
	{
	     for (int j = 1; j < count; j++)
		{
		    COM[i][j] /= single[j];
		}
	}
 }


void dijkstra(int traverse)
{
   long p=0,i,j,i1,j1;
   point val;
   int endx=COM[0][valg],endy=COM[1][valg];
   dist[val.x][val.y]=0;
   double fn=0;
   while(p<traverse)
   {
       p++;
       int min=infinity;
       for (i=0;i<binary.rows;i++)
       {
          for (j=0;j<binary.cols;j++)
          {
              fn=dist[i][j]+sqrt((endx-i)*(endx-i) +(endy-j)*(endy-j));
              if (fn<min && visited[i][j]==0)
              {

                 min=fn;
                 val.x=i;
                 val.y=j;
              }
          }
       }
       visited[val.x][val.y]=1;
       if (val.x==COM[0][valg] && val.y==COM[1][valg]) break;
       double mini;
       for (i1=-1;i1<=1;i1++)
       {
          for (j1=-1;j1<=1;j1++)
          {
             if (isvalid(binary,val.x+i1,val.y+j1)==1)
             {
                if (i1 % 2 == 1 && j1 % 2 == 1)
		    mini = dist[val.x][val.y] + sqrt(2.0);
		else
               	    mini = dist[val.x][val.y] + 1;
	        if (mini < dist[val.x + i1][val.y + j1])
	              {
	                    double gn=sqrt(i1*i1 + j1*j1);
		            dist[val.x + i1][val.y + j1] = mini + gn;
                            pt[val.x + i1][val.y + j1].x = val.x;
		            pt[val.x + i1][val.y + j1].y = val.y;
		      }
             }
          }
       }
   }
}




int main(int argc, char **argv)
 {

   Mat img = imread("ps1.jpg",1);
   Mat gray=imread("ps1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
   binary = Bin(gray);
   initialize();
   int count=DFSutil();
   centre(count);
   find(img);

   int i,j,val=0;
   for (i=0;i<max;i++)
   {
      for (j=0;j<max;j++)
      {
          if (visited[i][j]==0 || visited[i][j]==1 || visited[i][j]==10) {val++;visited[i][j]=0;}
          else visited[i][j]=-1;
      }
   }

   dijkstra(val);
   i=COM[0][valg];
   j=COM[1][valg];
   while(pt[i][j].x!=COM[0][valr] && pt[i][j].y!= COM[1][valr])
   {
        img.at<Vec3b>(i,j)[0]=255;
        img.at<Vec3b>(i,j)[1]=0;
        img.at<Vec3b>(i,j)[2]=0;
        visited[i][j]=2;
        i=pt[i][j].x;
        j=pt[i][j].y;
   }
   imshow("NEW",img);
   int iKey = waitKey(50);
   waitKey(0);
   return 0;
 }


