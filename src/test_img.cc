#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"


int main() {
  cv::namedWindow("IMG");

  cv::Mat tmp, tmp1;
  cv::Mat img = cv::imread("img.jpg");
  cv::imshow("IMG", img);
  cv::waitKey(0);

  cv::cvtColor(img, tmp, CV_BGR2HSV);
  cv::cvtColor(img, tmp1, CV_BGR2GRAY);

  std::vector<int> u, v;

  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      uint16_t h = tmp.at<cv::Vec3b>(i, j)[0] * 2;
      // green
      if ((h > 92 && h < 186) || (j < 110 && i < 220)) {
        tmp1.at<uint8_t>(i, j) = 0;
      } else {
        tmp1.at<uint8_t>(i, j) = 255;
        u.push_back(j);
        v.push_back(i);
      }
    }
  }

  cv::Mat pca_data(u.size(), 2, CV_64FC1);
  for (size_t i = 0; i < u.size(); i++) {
    pca_data.at<double>(i, 0) = u.at(i);
    pca_data.at<double>(i, 1) = v.at(i);
  }

  cv::PCA pca_analysis(pca_data, cv::Mat(), CV_PCA_DATA_AS_ROW);

  cv::Point cntr(
      static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

  std::cout << pca_analysis.mean.at<double>(0, 0) << ", "
            << pca_analysis.mean.at<double>(0, 1) << "\n";

  std::vector<cv::Point2d> eigen_vecs(2);
  std::vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i) {
    eigen_vecs[i] =
        cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                    pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);

    std::cout << pca_analysis.eigenvectors.at<double>(i, 0) << ", "
              << pca_analysis.eigenvectors.at<double>(i, 1) << ", "
              << eigen_val[i] << "\n";
  }

  cv::circle(img, cntr, 3, cv::Scalar(255, 0, 255), 2);
  cv::Point p1 = cntr + 0.1 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  cv::Point p2 = cntr + 0.1 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));

  cv::circle(img, p1, 3, cv::Scalar(0, 0, 255), 2);
  cv::circle(img, p2, 3, cv::Scalar(0, 255, 0), 2);

  cv::imshow("IMG", img);
  cv::waitKey(0);


  // cv::drawAxis(img, cntr, p1, cv::Scalar(0, 255, 0), 1);
  // cv::drawAxis(img, cntr, p2, cv::Scalar(255, 255, 0), 5);

  /*
  int dilation_size = 1;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
      cv::Point( dilation_size, dilation_size ) );
  // cv::dilate(tmp1, tmp, element);
  */

  /*
  cv::imshow("IMG", tmp1);
  cv::waitKey(0);

  //////////////
  cv::SimpleBlobDetector::Params params;
  params.filterByArea = false; true;
  params.minArea = 100;

  params.filterByCircularity = false;
  params.filterByColor = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;

  params.minThreshold = 0;
  params.maxThreshold = 210;

  cv::SimpleBlobDetector detector(params);

  // Detect blobs.
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(tmp1, keypoints);

  std::cout << keypoints.size() << "\n";

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  cv::Mat im_with_keypoints;
  cv::drawKeypoints(tmp1, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  // Show blobs
  cv::imshow("keypoints", im_with_keypoints );
  cv::waitKey(0);
  */

  /*
  cv::blur(tmp, tmp1, cv::Size(6, 6));
  cv::imshow("IMG", tmp1);
  cv::waitKey(0);

  cv::Canny(tmp1, tmp, 30, 30 * 3, 3, true);
  cv::imshow("IMG", tmp);
  cv::waitKey(0);
  */

  return 0;
}
