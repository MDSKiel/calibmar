#include "report.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>

namespace {
  std::vector<std::string> Split(const std::string& input, const std::string& regex) {
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator first{input.begin(), input.end(), re, -1}, last;
    return {first, last};
  }
}

namespace calibmar {
  namespace report {
    void WriteCalibrationReport(const std::string& path, const Calibration& calibration) {
      std::ofstream out_file(path);
      GenerateResultString(out_file, calibration);
      out_file.close();
    }

    void WriteCalibrationYaml(const std::string& path, const Calibration& calibration) {
      std::ofstream out_file(path);
      GenerateCalibrationYaml(out_file, calibration);
      out_file.close();
    }

    void GenerateCalibrationYaml(std::ostream& stream, const Calibration& calibration) {
      const colmap::Camera& camera = calibration.Camera();
      // model
      stream << "model: " << camera.ModelName() << std::endl;
      stream << "# " << camera.ParamsInfo() << std::endl;
      // parameters
      stream << "parameters: [";
      const std::vector<double>& params = camera.Params();
      std::copy(params.begin(), params.end() - 1, std::ostream_iterator<double>(stream, ", "));
      stream << *(params.end() - 1);
      stream << "]" << std::endl;
      // std dev parameters
      const std::vector<double>& intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
      if (intrinsics_std_dev.size() > 0) {
        stream << "# params_est_std_dev: ";
        std::copy(intrinsics_std_dev.begin(), intrinsics_std_dev.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(intrinsics_std_dev.end() - 1) << std::endl;
      }
      // non_svp
      std::string non_svp_model_name = camera.IsCameraRefractive() ? camera.RefracModelName() : "NONE";
      stream << "non_svp_model: " << non_svp_model_name << std::endl;
      if (camera.IsCameraRefractive()) {
        // one of the info strings contains a new line...
        std::string params_info = std::regex_replace(camera.RefracParamsInfo(), std::regex("\\n"), " ");
        stream << "# " << params_info << std::endl;
      }
      stream << "non_svp_parameters: [";
      if (camera.IsCameraRefractive()) {
        const std::vector<double>& non_svp_params = camera.RefracParams();
        std::copy(non_svp_params.begin(), non_svp_params.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(non_svp_params.end() - 1);
      }
      stream << "]" << std::endl;
      // std dev parameters non svp
      const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
      if (housing_std_dev.size() > 0) {
        stream << "# non_svp_est_std_dev: ";
        std::copy(housing_std_dev.begin(), housing_std_dev.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(housing_std_dev.end() - 1) << std::endl;
      }
      // width & height
      stream << "width: " << camera.Width() << std::endl;
      stream << "height: " << camera.Height() << std::endl;
      // overall RMS
      stream << "# overall_rms: " << calibration.CalibrationRms() << std::endl;
      // target
      stream << "# target: " << calibration.GetCalibrationTargetInfo();
    }

    std::string GenerateResultString(const Calibration& calibration) {
      std::stringstream result_stream;
      GenerateResultString(result_stream, calibration);
      return result_stream.str();
    }

    void GenerateResultString(std::ostream& stream, const Calibration& calibration) {
      int first_column_width = 22;
      int field_width = 13;
      const colmap::Camera& camera = calibration.Camera();
      // camera model
      std::string header = camera.IsCameraRefractive() ? "Camera & Housing Model:" : "Camera Model:";
      stream << header << std::endl << camera.ModelName();
      if (camera.IsCameraRefractive()) {
        stream << " " << camera.RefracModelName();
      }
      stream << std::endl << std::endl;
      // width & height
      stream << "Width & Height:" << std::endl;
      stream << camera.Width() << " " << camera.Height();
      stream << std::endl << std::endl;
      // camera matrix
      stream << "Camera Matrix:" << std::endl << camera.CalibrationMatrix() << std::endl << std::endl;
      // parameter lables
      std::vector<std::string> param_names = Split(camera.ParamsInfo(), ", ");
      stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Parameters:";
      for (auto& value : param_names) {
        stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      }
      stream << std::endl;
      // parameter values
      const std::vector<double>& params = camera.Params();
      stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
      for (auto& value : params) {
        stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      }
      stream << std::endl;
      // parameter std dev
      const std::vector<double>& intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
      if (intrinsics_std_dev.size() > 0) {
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
        for (auto& value : intrinsics_std_dev) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
      }
      if (camera.IsCameraRefractive()) {
        stream << std::endl;
        // housing labels
        std::vector<std::string> housing_param_names = Split(camera.RefracParamsInfo(), ", ");
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Housing Parameters:";
        for (auto& value : housing_param_names) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        // housing values
        const std::vector<double>& housing_params = camera.RefracParams();
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
        for (auto& value : housing_params) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        // housing std dev
        const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
        if (housing_std_dev.size() > 0) {
          stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
          for (auto& value : housing_std_dev) {
            stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
          }
          stream << std::endl;
        }
      }
      // overall rms
      stream << std::endl << "Overall RMS: " << calibration.CalibrationRms();
    }
  }

  namespace {
    bool TryParse(double* d, const std::string& string, size_t* chars_used) {
      try {
        *d = std::stod(string, chars_used);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    bool TryParse(int* i, const std::string& string, size_t* chars_used) {
      try {
        *i = std::stoi(string, chars_used);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    bool StartsWith(const std::string& string, const std::string& target) {
      return string.find(target) == 0;
    }

    inline void TrimEnd(std::string& s) {
      s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
    }
  }

  ImportedParameters ImportedParameters::ImportFromYaml(const std::string& path) {
    ImportedParameters parameters;

    std::filesystem::path file_path(path);
    if (!std::filesystem::exists(file_path) || !file_path.has_filename()) {
      return parameters;
    }

    std::ifstream input(path);

    parameters = ImportFromYaml(input);
    parameters.directory = file_path.parent_path().string();

    return parameters;
  }

  ImportedParameters ImportedParameters::ImportFromYaml(std::istream& stream) {
    ImportedParameters parameters;
    std::string line;
    while (std::getline(stream, line)) {
      TrimEnd(line);
      if (StartsWith(line, "model:")) {
        line.erase(0, sizeof("model: ") - 1);
        for (auto& [type, model] : CameraModel::CameraModels()) {
          if (model.model_name == line) {
            parameters.camera_model = type;
            break;
          }
        }
      }
      else if (StartsWith(line, "parameters: [")) {
        line.erase(0, sizeof("parameters: [") - 1);
        double p;
        size_t chars_used;
        while (TryParse(&p, line, &chars_used)) {
          parameters.camera_parameters.push_back(p);
          // + the comma or closing bracket
          line.erase(0, chars_used + 1);
        }
      }
      else if (StartsWith(line, "non_svp_model:")) {
        line.erase(0, sizeof("non_svp_model: ") - 1);
        for (auto& [type, model] : HousingInterface::HousingInterfaces()) {
          if (model.model_name == line) {
            parameters.housing_model = type;
            break;
          }
        }
      }
      else if (StartsWith(line, "non_svp_parameters: [")) {
        line.erase(0, sizeof("non_svp_parameters: [") - 1);
        double p;
        size_t chars_used;
        while (TryParse(&p, line, &chars_used)) {
          parameters.housing_parameters.push_back(p);
          // + the comma or closing bracket
          line.erase(0, chars_used + 1);
        }
      }
      else if (StartsWith(line, "# target: chessboard,")) {
        line.erase(0, sizeof("# target: chessboard,") - 1);
        int i;
        size_t chars_used;
        if (TryParse(&i, line, &chars_used)) {
          parameters.chessboard_columns = i;
          line.erase(0, chars_used + 1);
          if (TryParse(&i, line, &chars_used)) {
            parameters.chessboard_rows = i;
            line.erase(0, chars_used + 1);
            double d;
            if (TryParse(&d, line, &chars_used)) {
              parameters.square_size = d;
            }
          }
        }
      }
    }

    return parameters;
  }
}
