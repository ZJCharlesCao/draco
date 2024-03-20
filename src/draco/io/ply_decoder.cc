
// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "draco/io/ply_decoder.h"

#include "draco/core/macros.h"
#include "draco/core/status.h"
#include "draco/io/file_utils.h"
#include "draco/io/ply_property_reader.h"

namespace draco {
namespace {
int64_t CountNumTriangles(const PlyElement &face_element,
                          const PlyProperty &vertex_indices) {
  int64_t num_triangles = 0;
  for (int i = 0; i < face_element.num_entries(); ++i) {
    const int64_t list_size = vertex_indices.GetListEntryNumValues(i);
    if (list_size < 3) {
      // Correctly encoded ply files don't have less than three vertices.
      continue;
    }
    num_triangles += list_size - 2;
  }
  return num_triangles;
}
}  // namespace

PlyDecoder::PlyDecoder() : out_mesh_(nullptr), out_point_cloud_(nullptr) {}

Status PlyDecoder::DecodeFromFile(const std::string &file_name,
                                  Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  return DecodeFromFile(file_name, static_cast<PointCloud *>(out_mesh));
}

Status PlyDecoder::DecodeFromFile(const std::string &file_name,
                                  PointCloud *out_point_cloud) {
  std::vector<char> data;
  if (!ReadFileToBuffer(file_name, &data)) {
    return Status(Status::DRACO_ERROR, "Unable to read input file.");
  }
  buffer_.Init(data.data(), data.size());
  return DecodeFromBuffer(&buffer_, out_point_cloud);
}

Status PlyDecoder::DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh) {
  out_mesh_ = out_mesh;
  return DecodeFromBuffer(buffer, static_cast<PointCloud *>(out_mesh));
}

Status PlyDecoder::DecodeFromBuffer(DecoderBuffer *buffer,
                                    PointCloud *out_point_cloud) {
  out_point_cloud_ = out_point_cloud;
  buffer_.Init(buffer->data_head(), buffer->remaining_size());
  return DecodeInternal();
}

Status PlyDecoder::DecodeInternal() {
  PlyReader ply_reader;
  DRACO_RETURN_IF_ERROR(ply_reader.Read(buffer()));
  // First, decode the connectivity data.
  if (out_mesh_)
    DRACO_RETURN_IF_ERROR(DecodeFaceData(ply_reader.GetElementByName("face")));
  // Decode all attributes.
  DRACO_RETURN_IF_ERROR(
      DecodeVertexData(ply_reader.GetElementByName("vertex")));
  // In case there are no faces this is just a point cloud which does
  // not require deduplication.
  if (out_mesh_ && out_mesh_->num_faces() != 0) {
#ifdef DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED
    if (!out_point_cloud_->DeduplicateAttributeValues()) {
      return Status(Status::DRACO_ERROR,
                    "Could not deduplicate attribute values");
    }
#endif
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
    out_point_cloud_->DeduplicatePointIds();
#endif
  }
  return OkStatus();
}

Status PlyDecoder::DecodeFaceData(const PlyElement *face_element) {
  // We accept point clouds now.
  if (face_element == nullptr) {
    return OkStatus();
  }
  const PlyProperty *vertex_indices =
      face_element->GetPropertyByName("vertex_indices");
  if (vertex_indices == nullptr) {
    // The property name may be named either "vertex_indices" or "vertex_index".
    vertex_indices = face_element->GetPropertyByName("vertex_index");
  }
  if (vertex_indices == nullptr || !vertex_indices->is_list()) {
    return Status(Status::DRACO_ERROR, "No faces defined");
  }

  // Allocate faces.
  out_mesh_->SetNumFaces(CountNumTriangles(*face_element, *vertex_indices));
  const int64_t num_polygons = face_element->num_entries();

  PlyPropertyReader<PointIndex::ValueType> vertex_index_reader(vertex_indices);
  Mesh::Face face;
  FaceIndex face_index(0);
  for (int i = 0; i < num_polygons; ++i) {
    const int64_t list_offset = vertex_indices->GetListEntryOffset(i);
    const int64_t list_size = vertex_indices->GetListEntryNumValues(i);
    if (list_size < 3) {
      continue;  // All invalid polygons are skipped.
    }

    // Triangulate polygon assuming the polygon is convex.
    const int64_t num_triangles = list_size - 2;
    face[0] = vertex_index_reader.ReadValue(static_cast<int>(list_offset));
    for (int64_t ti = 0; ti < num_triangles; ++ti) {
      for (int64_t c = 1; c < 3; ++c) {
        face[c] = vertex_index_reader.ReadValue(
            static_cast<int>(list_offset + ti + c));
      }
      out_mesh_->SetFace(face_index, face);
      face_index++;
    }
  }
  out_mesh_->SetNumFaces(face_index.value());
  return OkStatus();
}

template <typename DataTypeT>
bool PlyDecoder::ReadPropertiesToAttribute(
    const std::vector<const PlyProperty *> &properties,
    PointAttribute *attribute, int num_vertices) {
  std::vector<std::unique_ptr<PlyPropertyReader<DataTypeT>>> readers;
  readers.reserve(properties.size());
  for (int prop = 0; prop < properties.size(); ++prop) {
    readers.push_back(std::unique_ptr<PlyPropertyReader<DataTypeT>>(
        new PlyPropertyReader<DataTypeT>(properties[prop])));
  }
  std::vector<DataTypeT> memory(properties.size());
  for (PointIndex::ValueType i = 0; i < static_cast<uint32_t>(num_vertices);
       ++i) {
    for (int prop = 0; prop < properties.size(); ++prop) {
      memory[prop] = readers[prop]->ReadValue(i);
    }
    attribute->SetAttributeValue(AttributeValueIndex(i), memory.data());
  }
  return true;
}

Status PlyDecoder::DecodeVertexData(const PlyElement *vertex_element) {
  if (vertex_element == nullptr) {
    return Status(Status::INVALID_PARAMETER, "vertex_element is null");
  }
  // TODO(b/34330853): For now, try to load x,y,z vertices, red,green,blue,alpha
  // colors, and nx,ny,nz normals. We need to add other properties later.
  const PlyProperty *const x_prop = vertex_element->GetPropertyByName("x");
  const PlyProperty *const y_prop = vertex_element->GetPropertyByName("y");
  const PlyProperty *const z_prop = vertex_element->GetPropertyByName("z");
  if (!x_prop || !y_prop || !z_prop) {
    // Currently, we require 3 vertex coordinates (this should be generalized
    // later on).
    return Status(Status::INVALID_PARAMETER, "x, y, or z property is missing");
  }
  const PointIndex::ValueType num_vertices = vertex_element->num_entries();
  out_point_cloud_->set_num_points(num_vertices);
  // Decode vertex positions.
  {
    // All properties must have the same type.
    if (x_prop->data_type() != y_prop->data_type() ||
        y_prop->data_type() != z_prop->data_type()) {
      return Status(Status::INVALID_PARAMETER,
                    "x, y, and z properties must have the same type");
    }
    // TODO(ostava): For now assume the position types are float32 or int32.
    const DataType dt = x_prop->data_type();
    if (dt != DT_FLOAT32 && dt != DT_INT32) {
      return Status(Status::INVALID_PARAMETER,
                    "x, y, and z properties must be of type float32 or int32");
    }

    GeometryAttribute va;
    va.Init(GeometryAttribute::POSITION, nullptr, 3, dt, false,
            DataTypeLength(dt) * 3, 0);
    const int att_id = out_point_cloud_->AddAttribute(va, true, num_vertices);
    std::vector<const PlyProperty *> properties;
    properties.push_back(x_prop);
    properties.push_back(y_prop);
    properties.push_back(z_prop);
    if (dt == DT_FLOAT32) {
      ReadPropertiesToAttribute<float>(
          properties, out_point_cloud_->attribute(att_id), num_vertices);
    } else if (dt == DT_INT32) {
      ReadPropertiesToAttribute<int32_t>(
          properties, out_point_cloud_->attribute(att_id), num_vertices);
    }
  }

  // Decode normals if present.
  const PlyProperty *const n_x_prop = vertex_element->GetPropertyByName("nx");
  const PlyProperty *const n_y_prop = vertex_element->GetPropertyByName("ny");
  const PlyProperty *const n_z_prop = vertex_element->GetPropertyByName("nz");
  if (n_x_prop != nullptr && n_y_prop != nullptr && n_z_prop != nullptr) {
    // For now, all normal properties must be set and of type float32
    if (n_x_prop->data_type() == DT_FLOAT32 &&
        n_y_prop->data_type() == DT_FLOAT32 &&
        n_z_prop->data_type() == DT_FLOAT32) {
      PlyPropertyReader<float> x_reader(n_x_prop);
      PlyPropertyReader<float> y_reader(n_y_prop);
      PlyPropertyReader<float> z_reader(n_z_prop);
      GeometryAttribute va;
      va.Init(GeometryAttribute::NORMAL, nullptr, 3, DT_FLOAT32, false,
              sizeof(float) * 3, 0);
      const int att_id = out_point_cloud_->AddAttribute(va, true, num_vertices);
      for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
        std::array<float, 3> val;
        val[0] = x_reader.ReadValue(i);
        val[1] = y_reader.ReadValue(i);
        val[2] = z_reader.ReadValue(i);
        out_point_cloud_->attribute(att_id)->SetAttributeValue(
            AttributeValueIndex(i), &val[0]);
      }
    }
  }


    // Decode fdcs if present.
   const PlyProperty *const fdc0_prop =vertex_element->GetPropertyByName("f_dc_0"); 
   const PlyProperty *const fdc1_prop = vertex_element->GetPropertyByName("f_dc_1"); 
   const PlyProperty *const fdc2_prop = vertex_element->GetPropertyByName("f_dc_2"); 
   if (fdc0_prop != nullptr && fdc1_prop != nullptr && fdc2_prop != nullptr) {
     // For now, all fdc properties must be set and of type float32
     if (fdc0_prop->data_type() == DT_FLOAT32 &&
         fdc1_prop->data_type() == DT_FLOAT32 &&
         fdc2_prop->data_type() == DT_FLOAT32) {
       PlyPropertyReader<float> fdc0_reader(fdc0_prop);
       PlyPropertyReader<float> fdc1_reader(fdc1_prop);
       PlyPropertyReader<float> fdc2_reader(fdc2_prop);
       GeometryAttribute va;
       va.Init(GeometryAttribute::FDC, nullptr, 3, DT_FLOAT32, false,
               sizeof(float) * 3, 0);
       const int att_id = out_point_cloud_->AddAttribute(va, true,
       num_vertices); for (PointIndex::ValueType i = 0; i < num_vertices; ++i)
       {
         std::array<float, 3> val;
         val[0] = fdc0_reader.ReadValue(i);
         val[1] = fdc1_reader.ReadValue(i);
         val[2] = fdc2_reader.ReadValue(i);
         out_point_cloud_->attribute(att_id)->SetAttributeValue(
             AttributeValueIndex(i), &val[0]);
       }
     }
   }
  ////Decode frests if present
  // const PlyProperty *const f_rest_0_prop = vertex_element->GetPropertyByName("f_rest_0");
  // const PlyProperty *const f_rest_1_prop = vertex_element->GetPropertyByName("f_rest_1");
  // const PlyProperty *const f_rest_2_prop = vertex_element->GetPropertyByName("f_rest_2");
  // if (f_rest_0_prop != nullptr && f_rest_1_prop != nullptr &&
  //     f_rest_2_prop != nullptr) {
  //   if (f_rest_0_prop->data_type() == DT_FLOAT32 &&
  //       f_rest_1_prop->data_type() == DT_FLOAT32 &&
  //       f_rest_2_prop->data_type() == DT_FLOAT32) {
  //     PlyPropertyReader<float> f_rest_0_reader(f_rest_0_prop);
  //     PlyPropertyReader<float> f_rest_1_reader(f_rest_1_prop);
  //     PlyPropertyReader<float> f_rest_2_reader(f_rest_2_prop);
  //     GeometryAttribute va;
  //     va.Init(GeometryAttribute::FREST, nullptr, 3, DT_FLOAT32, false,
  //             sizeof(float) * 3, 0);
  //     const int att_id = out_point_cloud_->AddAttribute(va, true,
  //                                                      num_vertices);
  //     for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
  //       std::array<float, 3> val;
  //       val[0] = f_rest_0_reader.ReadValue(i);
  //       val[1] = f_rest_1_reader.ReadValue(i);
  //       val[2] = f_rest_2_reader.ReadValue(i);
  //       out_point_cloud_->attribute(att_id)->SetAttributeValue(
  //           AttributeValueIndex(i), &val[0]);
  //     }
  //   }
  // }

      //Decode f_rest_0 to f_rest_44 if present.
    const PlyProperty *const f_rest_0_prop = vertex_element->GetPropertyByName("f_rest_0"); 
    const PlyProperty *const f_rest_1_prop = vertex_element->GetPropertyByName("f_rest_1"); 
    const PlyProperty *const f_rest_2_prop = vertex_element->GetPropertyByName("f_rest_2");
    const PlyProperty *const f_rest_3_prop = vertex_element->GetPropertyByName("f_rest_3");
    const PlyProperty *const f_rest_4_prop = vertex_element->GetPropertyByName("f_rest_4");
    const PlyProperty *const f_rest_5_prop = vertex_element->GetPropertyByName("f_rest_5");
    const PlyProperty *const f_rest_6_prop = vertex_element->GetPropertyByName("f_rest_6");
    const PlyProperty *const f_rest_7_prop = vertex_element->GetPropertyByName("f_rest_7");
    const PlyProperty *const f_rest_8_prop = vertex_element->GetPropertyByName("f_rest_8");
    const PlyProperty *const f_rest_9_prop = vertex_element->GetPropertyByName("f_rest_9");
    const PlyProperty *const f_rest_10_prop = vertex_element->GetPropertyByName("f_rest_10");
    const PlyProperty *const f_rest_11_prop = vertex_element->GetPropertyByName("f_rest_11");
    const PlyProperty *const f_rest_12_prop = vertex_element->GetPropertyByName("f_rest_12");
    const PlyProperty *const f_rest_13_prop = vertex_element->GetPropertyByName("f_rest_13");
    const PlyProperty *const f_rest_14_prop = vertex_element->GetPropertyByName("f_rest_14");
    const PlyProperty *const f_rest_15_prop = vertex_element->GetPropertyByName("f_rest_15");
    const PlyProperty *const f_rest_16_prop = vertex_element->GetPropertyByName("f_rest_16");
    const PlyProperty *const f_rest_17_prop = vertex_element->GetPropertyByName("f_rest_17");
    const PlyProperty *const f_rest_18_prop = vertex_element->GetPropertyByName("f_rest_18");
    const PlyProperty *const f_rest_19_prop = vertex_element->GetPropertyByName("f_rest_19");
    const PlyProperty *const f_rest_20_prop = vertex_element->GetPropertyByName("f_rest_20");   
    const PlyProperty *const f_rest_21_prop = vertex_element->GetPropertyByName("f_rest_21");
    const PlyProperty *const f_rest_22_prop = vertex_element->GetPropertyByName("f_rest_22");
    const PlyProperty *const f_rest_23_prop =
        vertex_element->GetPropertyByName("f_rest_23");
    const PlyProperty *const f_rest_24_prop =
        vertex_element->GetPropertyByName("f_rest_24");
    const PlyProperty *const f_rest_25_prop =
        vertex_element->GetPropertyByName("f_rest_25");
    const PlyProperty *const f_rest_26_prop =
        vertex_element->GetPropertyByName("f_rest_26");
    const PlyProperty *const f_rest_27_prop =
        vertex_element->GetPropertyByName("f_rest_27");
    const PlyProperty *const f_rest_28_prop =
        vertex_element->GetPropertyByName("f_rest_28");
    const PlyProperty *const f_rest_29_prop =
        vertex_element->GetPropertyByName("f_rest_29");
    const PlyProperty *const f_rest_30_prop =
        vertex_element->GetPropertyByName("f_rest_30");
    const PlyProperty *const f_rest_31_prop =
        vertex_element->GetPropertyByName("f_rest_31");
    const PlyProperty *const f_rest_32_prop =
        vertex_element->GetPropertyByName("f_rest_32");
    const PlyProperty *const f_rest_33_prop =
        vertex_element->GetPropertyByName("f_rest_33");
    const PlyProperty *const f_rest_34_prop =
        vertex_element->GetPropertyByName("f_rest_34");
    const PlyProperty *const f_rest_35_prop =
        vertex_element->GetPropertyByName("f_rest_35");
    const PlyProperty *const f_rest_36_prop =
        vertex_element->GetPropertyByName("f_rest_36");
    const PlyProperty *const f_rest_37_prop =
        vertex_element->GetPropertyByName("f_rest_37");
    const PlyProperty *const f_rest_38_prop =
        vertex_element->GetPropertyByName("f_rest_38");
    const PlyProperty *const f_rest_39_prop =
        vertex_element->GetPropertyByName("f_rest_39");
    const PlyProperty *const f_rest_40_prop =
        vertex_element->GetPropertyByName("f_rest_40");
    const PlyProperty *const f_rest_41_prop =
        vertex_element->GetPropertyByName("f_rest_41");
    const PlyProperty *const f_rest_42_prop =
        vertex_element->GetPropertyByName("f_rest_42");
    const PlyProperty *const f_rest_43_prop =
        vertex_element->GetPropertyByName("f_rest_43");
    const PlyProperty *const f_rest_44_prop =
        vertex_element->GetPropertyByName("f_rest_44");
    



    if (f_rest_0_prop != nullptr && f_rest_1_prop != nullptr && f_rest_2_prop != nullptr)

    {
        PlyPropertyReader<float> f_rest_0_reader(f_rest_0_prop);
        PlyPropertyReader<float> f_rest_1_reader(f_rest_1_prop);
        PlyPropertyReader<float> f_rest_2_reader(f_rest_2_prop);
        PlyPropertyReader<float> f_rest_3_reader(f_rest_3_prop);
        PlyPropertyReader<float> f_rest_4_reader(f_rest_4_prop);
        PlyPropertyReader<float> f_rest_5_reader(f_rest_5_prop);
        PlyPropertyReader<float> f_rest_6_reader(f_rest_6_prop);
        PlyPropertyReader<float> f_rest_7_reader(f_rest_7_prop);
        PlyPropertyReader<float> f_rest_8_reader(f_rest_8_prop);
        PlyPropertyReader<float> f_rest_9_reader(f_rest_9_prop);
        PlyPropertyReader<float> f_rest_10_reader(f_rest_10_prop);
        PlyPropertyReader<float> f_rest_11_reader(f_rest_11_prop);
        PlyPropertyReader<float> f_rest_12_reader(f_rest_12_prop);
        PlyPropertyReader<float> f_rest_13_reader(f_rest_13_prop);
        PlyPropertyReader<float> f_rest_14_reader(f_rest_14_prop);
        PlyPropertyReader<float> f_rest_15_reader(f_rest_15_prop);
        PlyPropertyReader<float> f_rest_16_reader(f_rest_16_prop);
        PlyPropertyReader<float> f_rest_17_reader(f_rest_17_prop);
        PlyPropertyReader<float> f_rest_18_reader(f_rest_18_prop);
        PlyPropertyReader<float> f_rest_19_reader(f_rest_19_prop);
        PlyPropertyReader<float> f_rest_20_reader(f_rest_20_prop);
        PlyPropertyReader<float> f_rest_21_reader(f_rest_21_prop);
        PlyPropertyReader<float> f_rest_22_reader(f_rest_22_prop);
        PlyPropertyReader<float> f_rest_23_reader(f_rest_23_prop);
        PlyPropertyReader<float> f_rest_24_reader(f_rest_24_prop);
        PlyPropertyReader<float> f_rest_25_reader(f_rest_25_prop);
        PlyPropertyReader<float> f_rest_26_reader(f_rest_26_prop);
        PlyPropertyReader<float> f_rest_27_reader(f_rest_27_prop);
        PlyPropertyReader<float> f_rest_28_reader(f_rest_28_prop);
        PlyPropertyReader<float> f_rest_29_reader(f_rest_29_prop);
        PlyPropertyReader<float> f_rest_30_reader(f_rest_30_prop);
        PlyPropertyReader<float> f_rest_31_reader(f_rest_31_prop);
        PlyPropertyReader<float> f_rest_32_reader(f_rest_32_prop);
        PlyPropertyReader<float> f_rest_33_reader(f_rest_33_prop);
        PlyPropertyReader<float> f_rest_34_reader(f_rest_34_prop);
        PlyPropertyReader<float> f_rest_35_reader(f_rest_35_prop);
        PlyPropertyReader<float> f_rest_36_reader(f_rest_36_prop);
        PlyPropertyReader<float> f_rest_37_reader(f_rest_37_prop);
        PlyPropertyReader<float> f_rest_38_reader(f_rest_38_prop);
        PlyPropertyReader<float> f_rest_39_reader(f_rest_39_prop);
        PlyPropertyReader<float> f_rest_40_reader(f_rest_40_prop);
        PlyPropertyReader<float> f_rest_41_reader(f_rest_41_prop);
        PlyPropertyReader<float> f_rest_42_reader(f_rest_42_prop);
        PlyPropertyReader<float> f_rest_43_reader(f_rest_43_prop);
        PlyPropertyReader<float> f_rest_44_reader(f_rest_44_prop);

        GeometryAttribute va;
        va.Init(GeometryAttribute::FREST, nullptr, 45, DT_FLOAT32, false,
                sizeof(float) * 45, 0);
        const int att_id = out_point_cloud_->AddAttribute(va, true,
                                                         num_vertices);
        for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
          std::array<float, 45> val;
          val[0] = f_rest_0_reader.ReadValue(i);
          val[1] = f_rest_1_reader.ReadValue(i);
          val[2] = f_rest_2_reader.ReadValue(i);
          val[3] = f_rest_3_reader.ReadValue(i);
          val[4] = f_rest_4_reader.ReadValue(i);
          val[5] = f_rest_5_reader.ReadValue(i);
          val[6] = f_rest_6_reader.ReadValue(i);
          val[7] = f_rest_7_reader.ReadValue(i);
          val[8] = f_rest_8_reader.ReadValue(i);
          val[9] = f_rest_9_reader.ReadValue(i);
          val[10] = f_rest_10_reader.ReadValue(i);
          val[11] = f_rest_11_reader.ReadValue(i);
          val[12] = f_rest_12_reader.ReadValue(i);
          val[13] = f_rest_13_reader.ReadValue(i);
          val[14] = f_rest_14_reader.ReadValue(i);
          val[15] = f_rest_15_reader.ReadValue(i);
          val[16] = f_rest_16_reader.ReadValue(i);
          val[17] = f_rest_17_reader.ReadValue(i);
          val[18] = f_rest_18_reader.ReadValue(i);
          val[19] = f_rest_19_reader.ReadValue(i);
          val[20] = f_rest_20_reader.ReadValue(i);
          val[21] = f_rest_21_reader.ReadValue(i);    
          val[22] = f_rest_22_reader.ReadValue(i);
          val[23] = f_rest_23_reader.ReadValue(i);
          val[24] = f_rest_24_reader.ReadValue(i);
          val[25] = f_rest_25_reader.ReadValue(i);
          val[26] = f_rest_26_reader.ReadValue(i);
          val[27] = f_rest_27_reader.ReadValue(i);
          val[28] = f_rest_28_reader.ReadValue(i);
          val[29] = f_rest_29_reader.ReadValue(i);
          val[30] = f_rest_30_reader.ReadValue(i);
          val[31] = f_rest_31_reader.ReadValue(i);
          val[32] = f_rest_32_reader.ReadValue(i);
          val[33] = f_rest_33_reader.ReadValue(i);
          val[34] = f_rest_34_reader.ReadValue(i);
          val[35] = f_rest_35_reader.ReadValue(i);
          val[36] = f_rest_36_reader.ReadValue(i);
          val[37] = f_rest_37_reader.ReadValue(i);
          val[38] = f_rest_38_reader.ReadValue(i);
          val[39] = f_rest_39_reader.ReadValue(i);
          val[40] = f_rest_40_reader.ReadValue(i);
          val[41] = f_rest_41_reader.ReadValue(i);
          val[42] = f_rest_42_reader.ReadValue(i);
          val[43] = f_rest_43_reader.ReadValue(i);
          val[44] = f_rest_44_reader.ReadValue(i);
          out_point_cloud_->attribute(att_id)->SetAttributeValue(
              AttributeValueIndex(i), &val[0]);
        }
     }
    

// Decode opacity if present
   const PlyProperty *const opacity_prop =
       vertex_element->GetPropertyByName("opacity");

   if (opacity_prop != nullptr) {
     if (opacity_prop->data_type() == DT_FLOAT32) {
       PlyPropertyReader<float> opacity_reader(opacity_prop);
       GeometryAttribute va;
       va.Init(GeometryAttribute::OPACITY, nullptr, 1, DT_FLOAT32, false,
               sizeof(float) * 1, 0);
       const int att_id =
           out_point_cloud_->AddAttribute(va, true, num_vertices);
       for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
         std::array<float, 1> val;
         val[0] = opacity_reader.ReadValue(i);
         out_point_cloud_->attribute(att_id)->SetAttributeValue(
             AttributeValueIndex(i), &val[0]);
       }
     }
   }


  // Decode scales if present
   const PlyProperty *const scale_0_prop =
       vertex_element->GetPropertyByName("scale_0");
   const PlyProperty *const scale_1_prop =
       vertex_element->GetPropertyByName("scale_1");
   const PlyProperty *const scale_2_prop =
       vertex_element->GetPropertyByName("scale_2");
   if (scale_0_prop != nullptr && scale_1_prop != nullptr &&
       scale_2_prop != nullptr) {
     if (scale_0_prop->data_type() == DT_FLOAT32 &&
         scale_1_prop->data_type() == DT_FLOAT32 &&
         scale_2_prop->data_type() == DT_FLOAT32) {
       PlyPropertyReader<float> scale_0_reader(scale_0_prop);
       PlyPropertyReader<float> scale_1_reader(scale_1_prop);
       PlyPropertyReader<float> scale_2_reader(scale_2_prop);
       GeometryAttribute va;
       va.Init(GeometryAttribute::SCALE, nullptr, 3, DT_FLOAT32, false,
               sizeof(float) * 3, 0);
       const int att_id =
           out_point_cloud_->AddAttribute(va, true, num_vertices);
       for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
         std::array<float, 3> val;
         val[0] = scale_0_reader.ReadValue(i);
         val[1] = scale_1_reader.ReadValue(i);
         val[2] = scale_2_reader.ReadValue(i);
         out_point_cloud_->attribute(att_id)->SetAttributeValue(
             AttributeValueIndex(i), &val[0]);
       }
     }
   }


    // Decode rots if present
    const PlyProperty *const rot_0_prop =
        vertex_element->GetPropertyByName("rot_0");
    const PlyProperty *const rot_1_prop =
        vertex_element->GetPropertyByName("rot_1");
    const PlyProperty *const rot_2_prop =
        vertex_element->GetPropertyByName("rot_2");
    const PlyProperty *const rot_3_prop =
        vertex_element->GetPropertyByName("rot_3");
    if (rot_0_prop != nullptr && rot_1_prop != nullptr &&
        rot_2_prop != nullptr && rot_3_prop != nullptr) {
        if (rot_0_prop->data_type() == DT_FLOAT32 &&
            rot_1_prop->data_type() == DT_FLOAT32 &&
            rot_2_prop->data_type() == DT_FLOAT32 &&
            rot_3_prop->data_type() == DT_FLOAT32) {
        PlyPropertyReader<float> rot_0_reader(rot_0_prop);
        PlyPropertyReader<float> rot_1_reader(rot_1_prop);
        PlyPropertyReader<float> rot_2_reader(rot_2_prop);
        PlyPropertyReader<float> rot_3_reader(rot_3_prop);
        GeometryAttribute va;
        va.Init(GeometryAttribute::ROT, nullptr, 4, DT_FLOAT32, false,
                sizeof(float) * 4, 0);
        const int att_id =
            out_point_cloud_->AddAttribute(va, true, num_vertices);
        for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
            std::array<float, 4> val;
            val[0] = rot_0_reader.ReadValue(i);
            val[1] = rot_1_reader.ReadValue(i);
            val[2] = rot_2_reader.ReadValue(i);
            val[3] = rot_3_reader.ReadValue(i);
            out_point_cloud_->attribute(att_id)->SetAttributeValue(
                AttributeValueIndex(i), &val[0]);
        }
        }
    }
       

  // Decode color data if present.
  int num_colors = 0;
  const PlyProperty *const r_prop = vertex_element->GetPropertyByName("red");
  const PlyProperty *const g_prop = vertex_element->GetPropertyByName("green");
  const PlyProperty *const b_prop = vertex_element->GetPropertyByName("blue");
  const PlyProperty *const a_prop = vertex_element->GetPropertyByName("alpha");
  if (r_prop) {
    ++num_colors;
  }
  if (g_prop) {
    ++num_colors;
  }
  if (b_prop) {
    ++num_colors;
  }
  if (a_prop) {
    ++num_colors;
  }

  if (num_colors) {
    std::vector<std::unique_ptr<PlyPropertyReader<uint8_t>>> color_readers;
    const PlyProperty *p;
    if (r_prop) {
      p = r_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'red' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }
    if (g_prop) {
      p = g_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'green' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }
    if (b_prop) {
      p = b_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'blue' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }
    if (a_prop) {
      p = a_prop;
      // TODO(ostava): For now ensure the data type of all components is uint8.
      DRACO_DCHECK_EQ(true, p->data_type() == DT_UINT8);
      if (p->data_type() != DT_UINT8) {
        return Status(Status::INVALID_PARAMETER,
                      "Type of 'alpha' property must be uint8");
      }
      color_readers.push_back(std::unique_ptr<PlyPropertyReader<uint8_t>>(
          new PlyPropertyReader<uint8_t>(p)));
    }

    GeometryAttribute va;
    va.Init(GeometryAttribute::COLOR, nullptr, num_colors, DT_UINT8, true,
            sizeof(uint8_t) * num_colors, 0);
    const int32_t att_id =
        out_point_cloud_->AddAttribute(va, true, num_vertices);
    for (PointIndex::ValueType i = 0; i < num_vertices; ++i) {
      std::array<uint8_t, 4> val;
      for (int j = 0; j < num_colors; j++) {
        val[j] = color_readers[j]->ReadValue(i);
      }
      out_point_cloud_->attribute(att_id)->SetAttributeValue(
          AttributeValueIndex(i), &val[0]);
    }
  }

  return OkStatus();
}

}  // namespace draco
