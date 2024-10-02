from model import *

def train_model(net, device, train_set, optimizer, epoch):
    net.train()
    for batch_index, (data, label) in enumerate(train_set):
        data, label = data.to(device), label.to(device)
        optimizer.zero_grad()
        output = net(data)
        loss = F.cross_entropy(output, label)
        loss.backward()
        optimizer.step()
        if batch_index % 3000 == 0:
            print("Train Epoch : {}\t Loss : {:.6f}".format(epoch, loss.item()))