// Copyright (c) 2024，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "feature_workflow.h"

// Function to calculate cosine similarity
Matrix cosine_similarity(const Matrix& features1, const Matrix& features2) {
    int m = features1.size();
    int n = features2.size();

    // Calculate norms
    std::vector<float> norm1(m, 0.0);
    std::vector<float> norm2(n, 0.0);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < features1[i].size(); ++j) {
            norm1[i] += features1[i][j] * features1[i][j];
        }
        norm1[i] = std::sqrt(norm1[i]);
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < features2[i].size(); ++j) {
            norm2[i] += features2[i][j] * features2[i][j];
        }
        norm2[i] = std::sqrt(norm2[i]);
    }

    // Normalize features
    Matrix features1_normalized(m, std::vector<float>(features1[0].size()));
    Matrix features2_normalized(n, std::vector<float>(features2[0].size()));
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < features1[i].size(); ++j) {
            features1_normalized[i][j] = features1[i][j] / norm1[i];
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < features2[i].size(); ++j) {
            features2_normalized[i][j] = features2[i][j] / norm2[i];
        }
    }

    // Calculate cosine similarity
    Matrix result(m, std::vector<float>(n, 0.0));
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            float dotProduct = 0.0;
            for (int k = 0; k < features1_normalized[i].size(); ++k) {
                dotProduct += features1_normalized[i][k] * features2_normalized[j][k];
            }
            result[i][j] = dotProduct;
        }
    }

    return result;
}

// Function to perform element-wise multiplication with a scalar
Matrix scalar_multiply(const Matrix& A, float scalar) {
    int m = A.size();
    int n = A[0].size();

    // Initialize result matrix with zeros
    Matrix result(m, std::vector<float>(n, 0.0));

    // Perform element-wise multiplication
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            result[i][j] = A[i][j] * scalar;
        }
    }

    return result;
}

// Function to calculate softmax
Matrix softmax(const Matrix& logits) {
    int rows = logits.size();
    int cols = logits[0].size();

    Matrix softmaxResult(rows, std::vector<float>(cols, 0.0));
    for (int i = 0; i < rows; ++i) {
        float sumExp = 0.0;
        for (int j = 0; j < cols; ++j) {
            sumExp += std::exp(logits[i][j]);
        }
        for (int j = 0; j < cols; ++j) {
            softmaxResult[i][j] = std::exp(logits[i][j]) / sumExp;
        }
    }
    return softmaxResult;
}

Matrix transpose(const Matrix& matrix) {
    int rows = matrix.size();
    int cols = matrix[0].size();

    // Create a new matrix with swapped dimensions
    Matrix transposed(cols, std::vector<float>(rows));

    // Perform transposition
    for (int i = 0; i < cols; ++i) {
        for (int j = 0; j < rows; ++j) {
            transposed[i][j] = matrix[j][i];
        }
    }
    return transposed;
}

float cosine_similarity(const float* data1, const float* data2) {
    float norm1 = 0.0f;
    float norm2 = 0.0f;
    float result = 0.0f;

    for (int i = 0; i < 512; ++i) {
        result += data1[i] * data2[i];
        norm1 += data1[i] * data1[i];
        norm2 += data2[i] * data2[i];
    }

    norm1 = std::sqrt(norm1);
    norm2 = std::sqrt(norm2);

    if (norm1 == 0 || norm2 == 0) {
        return 0.0f;
    }

    return result / (norm1 * norm2);
}