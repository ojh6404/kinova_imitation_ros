import os
import clip
import torch
import time
import os
import copy

import numpy as np
from sklearn.linear_model import LogisticRegression
from torch.utils.data import DataLoader, Dataset

from PIL import Image

from tqdm import tqdm

import torch.nn as nn
import torch.optim as optim
import torch

from sklearn.model_selection import train_test_split

def get_features(dataset):
    all_features = []
    all_labels = []

    with torch.no_grad():
        for images, labels in tqdm(DataLoader(dataset, batch_size=100)):
            features = model.encode_image(images.to(device))
            all_features.append(features)
            all_labels.append(labels)
    return torch.cat(all_features).cpu().numpy(), torch.cat(all_labels).cpu().numpy()

class CustomDataset1(Dataset):
    def __init__(self, data_dir, transform):
        self.data_dir = data_dir
        self.image_list = []
        for root, dirs, files in os.walk(data_dir):
            for file in files:
                if file.endswith('.jpg') or file.endswith('.jpeg') or file.endswith('.png'):
                    self.image_list.append(os.path.join(root, file))
        self.transform = transform

    def __len__(self):
        return len(self.image_list)

    def __getitem__(self, index):
        image_path = self.image_list[index]
        image = Image.open(image_path).convert('RGB')
        # Do any preprocessing or transformations here
        image = self.transform(image)
        #print(features.shape)

        label = int(os.path.basename(os.path.dirname(image_path)))

        # Return the image and any associated labels
        return image, label

class CustomDataset2(Dataset):
    def __init__(self, features, labels):
        self.features = features
        self.labels = labels
    def __len__(self):
        return len(self.features)
    def __getitem__(self, index):
        feature = self.features[index]
        label = self.labels[index]
        return torch.tensor(feature).to(device), torch.tensor(label).to(device)

class Linear(nn.Module):
    def __init__(self,input_size, output_size):
        super().__init__()
        self.fc1 = nn.Linear(input_size, output_size)

    def forward(self, x):
        # x = x.to(self.fc1.weight.dtype) # convert input tensor to the same dtype as weight tensor

        x = self.fc1(x)
        return x

def train_model(model, criterion, optimizer, num_epochs=25):
    since = time.time()

    best_model_wts = copy.deepcopy(model.state_dict())
    best_acc = 0.0

    for epoch in range(num_epochs):
        print(f'Epoch {epoch}/{num_epochs - 1}')
        print('-' * 10)

        # Each epoch has a training and validation phase
        for phase in ['train', 'val']:
            if phase == 'train':
                model.train()  # Set model to training mode
            else:
                model.eval()   # Set model to evaluate mode

            running_loss = 0.0
            running_corrects = 0

            # Iterate over data.
            for inputs, labels in dataloaders[phase]:
                inputs = inputs.to(device)
                labels = labels.to(device)

                # zero the parameter gradients
                optimizer.zero_grad()

                # forward
                # track history if only in train
                with torch.set_grad_enabled(phase == 'train'):
                    outputs = model(inputs)
                    _, preds = torch.max(outputs, 1)
                    loss = criterion(outputs, labels)

                    # backward + optimize only if in training phase
                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                # statistics
                running_loss += loss.item() * inputs.size(0)
                running_corrects += torch.sum(preds == labels.data)
            epoch_loss = running_loss / dataset_sizes[phase]
            epoch_acc = running_corrects.double() / dataset_sizes[phase]
            print(f'{phase} Loss: {epoch_loss:.4f} Acc: {epoch_acc:.4f}')

            # deep copy the model
            if phase == 'val' and epoch_acc > best_acc:
                best_acc = epoch_acc
                best_model_wts = copy.deepcopy(model.state_dict())
                if not os.path.exists('model'):
                    os.makedirs('model')
                torch.save(model.to("cpu"), 'model/best_model.pth')
                model.to(device)
    time_elapsed = time.time() - since
    print(f'Training complete in {time_elapsed // 60:.0f}m {time_elapsed % 60:.0f}s')
    print(f'Best val Acc: {best_acc:4f}')

    # load best model weights
    model.load_state_dict(best_model_wts)
    return model

batch_size=1
epochs = 10
# Load the model
device = "cuda" if torch.cuda.is_available() else "cpu"
#model, preprocess = clip.load('ViT-L/14@336px', device)
#model, preprocess = clip.load('ViT-L/14', device)
model, preprocess = clip.load('ViT-B/32', device)
# Load the dataset
dataset = CustomDataset1('data', preprocess)

# Calculate the image features
features, labels = get_features(dataset)
features = features.astype(np.float32)

# Split the dataset into training and testing subsets
train_features, test_features, train_labels, test_labels = train_test_split(
    features, labels, test_size=0.2, random_state=42)
# create train dataset and dataloader
train_dataset = CustomDataset2(train_features, train_labels)
train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=False)

# create test dataset and dataloader
test_dataset = CustomDataset2(test_features, test_labels)
test_dataloader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

dataloaders = {'train': train_dataloader, 'val': test_dataloader}
dataset_sizes = {'train': len(train_dataset), 'val': len(test_dataset) }
# learning one linear layer
model = Linear(features[0].shape[0], len(set(labels)))

model = model.to(device)

criterion = nn.CrossEntropyLoss()

# Observe that all parameters are being optimized
optimizer_ft = optim.Adam(model.parameters())

model = train_model(model, criterion, optimizer_ft,num_epochs=epochs)

print(model(next(iter(test_dataloader))[0]))
