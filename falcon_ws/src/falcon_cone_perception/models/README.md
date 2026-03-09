# Cone Perception Models

This directory contains the pre-trained machine learning models used for cone detection.

## YOLOv8 Model

- **Model File**: `best.pt`
- **Framework**: YOLOv8 (PyTorch)
- **Input Size**: 640x480
- **Classes**:
  - Yellow Cone (Class 0)
  - Blue Cone (Class 1)
  - Orange Cone (Class 2)

### Model Download

Download the trained model from the team's model repository or training pipeline output:

```bash
# Example download (replace with actual path)
wget https://path-to-model-repo/best.pt -O best.pt
```

### Model Training

To retrain the model with new data, use the YOLOv8 training pipeline:

```bash
# Install YOLOv8
pip install ultralytics

# Train model
yolo detect train data=data.yaml model=yolov8m.pt epochs=100 imgsz=640
```

## Model Specifications

- **Architecture**: YOLOv8 Medium (m)
- **Training Dataset**: EUFS/FSAE cone dataset
- **Performance Metrics**: [Add relevant metrics here]
  - mAP@0.5: XX%
  - Precision: XX%
  - Recall: XX%

## Usage

Models are loaded by the cone detection node. Configure the model path in `config/yolo_detector.yaml`.
