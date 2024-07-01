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

#include <cmath>
#include <vector>

// Define a 2D vector type
typedef std::vector<std::vector<float>> Matrix;

// Function to calculate cosine similarity
Matrix cosine_similarity(const Matrix& features1, const Matrix& features2);

// Function to perform element-wise multiplication with a scalar
Matrix scalar_multiply(const Matrix& A, float scalar);

// Function to calculate softmax
Matrix softmax(const Matrix& logits);

Matrix transpose(const Matrix& matrix);

float cosine_similarity(const float* data1, const float* data2);