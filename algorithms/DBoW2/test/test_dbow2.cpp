#include <gtest/gtest.h>
#include <iostream>
#include <vector>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

using namespace DBoW2;
using namespace std;

class MyTest : public ::testing::Test
{
protected:
    MyTest() {}
    ~MyTest() {}

    virtual void SetUp()
    {
    }
    virtual void TearDown() {}

public:
    const int NIMAGES = 4;
};

TEST_F(MyTest, DBoW2)
{
    vector<vector<cv::Mat>> features;
    features.reserve(NIMAGES);
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    for (int i = 0; i < NIMAGES; ++i)
    {
        stringstream ss;
        ss << "images/image" << i << ".png";

        cv::Mat image = cv::imread(ss.str(), 0);
        cv::Mat mask;
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        orb->detectAndCompute(image, mask, keypoints, descriptors);

        features.push_back(vector<cv::Mat>());
        features.back().resize(descriptors.rows);

        for (int i = 0; i < descriptors.rows; ++i)
        {
            features.back()[i] = descriptors.row(i);
        }
    }

    const int k = 9;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType scoring = L1_NORM;

    OrbVocabulary voc(k, L, weight, scoring);
    voc.create(features);

    BowVector v1, v2;
    for (int i = 0; i < NIMAGES; i++)
    {
        voc.transform(features[i], v1);
        for (int j = 0; j < NIMAGES; j++)
        {
            voc.transform(features[j], v2);
            double score = voc.score(v1, v2);
            if(i==j) EXPECT_GT(score, 0.999);
        }
    }

    // voc.save("small_voc.yml.gz");

    // OrbVocabulary voc("small_voc.yml.gz");

    OrbDatabase db(voc, false, 0); // false = do not use direct index
    // add images to the database
    for (int i = 0; i < NIMAGES; i++)
    {
        db.add(features[i]);
    }

    QueryResults ret;
    for (int i = 0; i < NIMAGES; i++)
    {
        db.query(features[i], ret, 4);
        EXPECT_EQ(i, static_cast<int>(ret[0].Id));
        EXPECT_GT(ret[0].Score, 0.999);
    }
}