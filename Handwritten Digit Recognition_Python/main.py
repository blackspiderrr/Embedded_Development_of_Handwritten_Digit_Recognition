from train import *
from test import *

#定义超参数
batch_size = 32
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
epochs = 10

#定义转换器
transformer = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.1307,), (0.3081,))])

#下载数据集
train_dataset = datasets.MNIST("data", train=True, download=True, transform=transformer)
test_dataset = datasets.MNIST("data", train=False, download=True, transform=transformer)

#加载数据集
train_loader = torch.utils.data.DataLoader(train_dataset, batch_size, shuffle=True)
test_loader = torch.utils.data.DataLoader(test_dataset, batch_size, shuffle=True)
#展示
fig = plt.figure()
for i in range(12):
    plt.subplot(3, 4, i+1)
    plt.tight_layout()
    plt.imshow(train_dataset.train_data[i], cmap='gray', interpolation='none')
    plt.title("Labels: {}".format(train_dataset.train_labels[i]))
    plt.xticks([])
    plt.yticks([])
plt.show()

#定义优化器
model = net().to(device)
optimizer = torch.optim.Adam(model.parameters())

#训练
acc_test = []
loss_test = []
for i in range(1, epochs+1):
    train_model(model, device, train_loader, optimizer, i)
    avg_loss, accuracy = test_model(model, device, test_loader)
    acc_test.append(accuracy)
    loss_test.append(avg_loss)

torch.save(model, 'model.pt')

plt.plot(acc_test)
plt.xlabel('Epoch')
plt.ylabel('Accuracy On TestSet')
plt.show()

plt.plot(loss_test)
plt.xlabel('Epoch')
plt.ylabel('Loss On TestSet')
plt.show()