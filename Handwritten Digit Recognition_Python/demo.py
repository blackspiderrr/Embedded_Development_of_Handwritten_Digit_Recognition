#可视化效果
from imports import *
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

transformer = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.1307,), (0.3081,))])
test_dataset = datasets.MNIST("data", train=False, download=True, transform=transformer)
test_loader = torch.utils.data.DataLoader(test_dataset, 32, shuffle=True)

model = torch.load('model.pt')

for i, (data, label) in enumerate(test_loader):
    data, label = data.to(device), label.to(device)
    if (i <= 9):
        image = data.cpu().numpy()[i].squeeze()
        output = model(data)
        output = output.argmax(dim=1)
        plt.imshow(image, cmap='gray', interpolation='none')
        plt.ion()
        plt.title("Prediction: {}".format(output[i].item()))
        plt.xticks([])
        plt.yticks([])
        plt.show()
        plt.pause(1)
        plt.close()
