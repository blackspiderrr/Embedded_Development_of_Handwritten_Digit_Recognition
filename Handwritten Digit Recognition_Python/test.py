from model import *

def test_model(net, device, test_set):
    net.eval()
    correct = 0
    loss = 0
    with torch.no_grad():
        for batch_index, (data, label) in enumerate(test_set):
            data, label = data.to(device), label.to(device)
            predict = net(data)
            loss += F.cross_entropy(predict, label).item()
            pred = predict.argmax(dim=1)
            correct += pred.eq(label.view_as(pred)).sum().item()
        avg_loss = loss / len(test_set.dataset)
        accuracy = 100.0 * correct / len(test_set.dataset)
        print("Test: Average_loss : {:.4f}\t Accuracy : {:.3f}".format(avg_loss, accuracy))
        return avg_loss, accuracy






