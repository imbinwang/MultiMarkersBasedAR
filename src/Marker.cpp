#include "../include/Marker.h"
#include "../include/DebugHelpers.h"

//#define SHOW_DEBUG_IMAGES

Marker::Marker()
	: m_id(-1)
{
	uchar marker213[15] = 		
	{
		1,0,0,0,0,
		1,0,1,1,1,
		0,1,1,1,0
	};
	std::vector<uchar> marker213word0(marker213 + 0*5, marker213 + 1*5);
	std::vector<uchar> marker213word1(marker213 + 1*5, marker213 + 2*5);
	std::vector<uchar> marker213word2(marker213 + 2*5, marker213 + 3*5);
	WordsInOneMarker marker213words;
	marker213words.push_back(marker213word0);
	marker213words.push_back(marker213word1);
	marker213words.push_back(marker213word2);
	m_markerWordsSet.push_back(marker213words);

	uchar marker231[25] = 		
	{
		1,0,0,0,1,
		0,1,0,1,1,
		1,1,1,0,0,
		1,0,1,1,1,
		0,1,1,1,0
	};
	std::vector<uchar> marker231word0(marker231 + 0*5, marker231 + 1*5);
	std::vector<uchar> marker231word1(marker231 + 1*5, marker231 + 2*5);
	std::vector<uchar> marker231word2(marker231 + 2*5, marker231 + 3*5);
	std::vector<uchar> marker231word3(marker231 + 3*5, marker231 + 4*5);
	std::vector<uchar> marker231word4(marker231 + 4*5, marker231 + 5*5);
	WordsInOneMarker marker231words;
	marker231words.push_back(marker231word0);
	marker231words.push_back(marker231word1);
	marker231words.push_back(marker231word2);
	marker231words.push_back(marker231word3);
	marker231words.push_back(marker231word4);
	m_markerWordsSet.push_back(marker231words);

	uchar marker411[25] = 		
	{
		1,0,0,1,0,
		0,1,1,0,1,
		1,0,1,1,0,
		1,1,1,0,1,
		0,1,0,1,1
	};
	std::vector<uchar> marker411word0(marker411 + 0*5, marker411 + 1*5);
	std::vector<uchar> marker411word1(marker411 + 1*5, marker411 + 2*5);
	std::vector<uchar> marker411word2(marker411 + 2*5, marker411 + 3*5);
	std::vector<uchar> marker411word3(marker411 + 3*5, marker411 + 4*5);
	std::vector<uchar> marker411word4(marker411 + 4*5, marker411 + 5*5);
	WordsInOneMarker marker411words;
	marker411words.push_back(marker411word0);
	marker411words.push_back(marker411word1);
	marker411words.push_back(marker411word2);
	marker411words.push_back(marker411word3);
	marker411words.push_back(marker411word4);
	m_markerWordsSet.push_back(marker411words);

	uchar marker561[25] = 		
	{
		1,1,0,0,1,
		0,0,1,0,0,
		0,1,0,1,0,
		0,0,1,0,1,
		1,0,0,1,0
	};
	std::vector<uchar> marker561word0(marker561 + 0*5, marker561 + 1*5);
	std::vector<uchar> marker561word1(marker561 + 1*5, marker561 + 2*5);
	std::vector<uchar> marker561word2(marker561 + 2*5, marker561 + 3*5);
	std::vector<uchar> marker561word3(marker561 + 3*5, marker561 + 4*5);
	std::vector<uchar> marker561word4(marker561 + 4*5, marker561 + 5*5);
	WordsInOneMarker marker561words;
	marker561words.push_back(marker561word0);
	marker561words.push_back(marker561word1);
	marker561words.push_back(marker561word2);
	marker561words.push_back(marker561word3);
	marker561words.push_back(marker561word4);
	m_markerWordsSet.push_back(marker561words);
}

bool operator<(const Marker &M1,const Marker&M2)
{
	return M1.m_id<M2.m_id;
}

/*
** rotate the matrix in clockwise by 90 degree
*/
cv::Mat Marker::rotate(cv::Mat in)
{
	cv::Mat out;
	in.copyTo(out);
	for (int i=0;i<in.rows;i++)
	{
		for (int j=0;j<in.cols;j++)
		{
			out.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);
		}
	}
	return out;
}

/*
** hamming distance to one possible marker
*/
int Marker::hammDistMarker(cv::Mat bits, WordsInOneMarker &possibleWords)
{
	int dist=0;

	for (int y=0;y<5;y++)
	{
		int minSum=1e5; //hamming distance to each possible word

		for (int p=0;p<(int)possibleWords.size();p++)
		{
			int sum=0;
			//now, count
			for (int x=0;x<5;x++)
			{
				sum += bits.at<uchar>(y,x) == possibleWords[p][x] ? 0 : 1;
			}

			if (minSum>sum)
				minSum=sum;
		}

		//do the and
		dist += minSum;
	}

	return dist;
}

int Marker::mat2id(const cv::Mat &bits)
{
	// bits consist of 5 words which are represented like this: 
	//		b b b b b
	//		b b b b b
	//		b b b b b
	//		b b b b b
	//		b b b b b
	// each row is a word, word: bit0 bit1 bit2 bit3 bit4
	// bit1 and bit3 are valid information bits
	// bit0, bit2 and bit4 are parity bits
	int val=0;
	for (int y=0;y<5;y++)
	{
		val<<=1;
		if ( bits.at<uchar>(y,1)) val|=1;
		val<<=1;
		if ( bits.at<uchar>(y,3)) val|=1;
	}
	return val;
}

int Marker::getMarkerId(cv::Mat &markerImage,int &nRotations)
{
	assert(markerImage.rows == markerImage.cols);
	assert(markerImage.type() == CV_8UC1);

	cv::Mat grey = markerImage.clone();

	// Threshold image
	cv::threshold(grey, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

#ifdef SHOW_DEBUG_IMAGES
	cv::showAndSave("Binary marker", grey);
#endif

	//Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
	//the external border should be entirely black

	int cellSize = markerImage.rows / 7;

	// check the border
	for (int y=0;y<7;y++)
	{
		int inc=6;

		//for first and last row, check the whole border
		if (y==0 || y==6) inc=1; 

		for (int x=0;x<7;x+=inc)
		{
			int cellX = x * cellSize;
			int cellY = y * cellSize;
			cv::Mat cell = grey(cv::Rect(cellX,cellY,cellSize,cellSize));

			int nZ = cv::countNonZero(cell);

			if (nZ > (cellSize*cellSize) / 2)
			{
				//can not be a marker because the border element is not black!
				return -1;
			}
		}
	}

	cv::Mat bitMatrix = cv::Mat::zeros(5,5,CV_8UC1);

	//get information(for each inner square, determine if it is black or white)  
	for (int y=0;y<5;y++)
	{
		for (int x=0;x<5;x++)
		{
			int cellX = (x+1)*cellSize;
			int cellY = (y+1)*cellSize;
			cv::Mat cell = grey(cv::Rect(cellX,cellY,cellSize,cellSize));

			int nZ = cv::countNonZero(cell);
			if (nZ> (cellSize*cellSize) /2) 
				bitMatrix.at<uchar>(y,x) = 1;
		}
	}

	// check all possible marker
	for(int i=0; i<(int)m_markerWordsSet.size();++i)
	{
		//check all possible rotations
		cv::Mat rotations[4];
		int distances[4];

		rotations[0] = bitMatrix;  
		distances[0] = hammDistMarker(rotations[0], m_markerWordsSet[i]);

		std::pair<int,int> minDist(distances[0],0);

		for (int j=1; j<4; j++)
		{
			//get the hamming distance to the nearest possible word
			rotations[j] = rotate(rotations[j-1]);
			distances[j] = hammDistMarker(rotations[j], m_markerWordsSet[i]);

			if (distances[j] < minDist.first)
			{
				minDist.first  = distances[j];
				minDist.second = j;
			}
		}

		nRotations = minDist.second;
		if (minDist.first == 0)
		{
			// when hamming distance == 0, that means the code is completely matched
			int xxx = mat2id(rotations[minDist.second]);
			return xxx;
		}
	}

	return -1;

}

void Marker::drawContour(cv::Mat& image, cv::Scalar color) const
{
	float thickness = 2;

	cv::line(image, m_points[0], m_points[1], color, thickness, CV_AA);
	cv::line(image, m_points[1], m_points[2], color, thickness, CV_AA);
	cv::line(image, m_points[2], m_points[3], color, thickness, CV_AA);
	cv::line(image, m_points[3], m_points[0], color, thickness, CV_AA);
}

/*
** generate a marker image by the assigned size and id
*/
void Marker::generateMarkerImg(const int cellSize, const std::vector<int> markerCode, 
		cv::Mat &markerImg)
{
	int markerCodeLength = (int)markerCode.size();
	assert(markerCodeLength >= 25);
	markerImg = cv::Mat::zeros(7*cellSize, 7*cellSize, CV_8UC3);

	// fill the marker by the id
	cv::Mat whiteCell(cellSize, cellSize, CV_8UC3, cv::Scalar::all(255));
	for (int y=0;y<5;y++)
	{
		for (int x=0;x<5;x++)
		{
			// for bit 1
			if(markerCode[5*y + x])
			{
				int cellX = (x+1)*cellSize;
				int cellY = (y+1)*cellSize;
				cv::Mat cell = markerImg(cv::Rect(cellX,cellY,cellSize,cellSize));
				whiteCell.copyTo(cell);
			}		
		}
	}
}
