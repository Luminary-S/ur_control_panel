import collections

#ref  有序字典: https://blog.csdn.net/chenmozhe22/article/details/90136849 

d1={}
d1=collections.OrderedDict()  #将普通字典转换为有序字典
if not d1:
    print('no')
d1['a']='A'
d1['b']='B'
d1['c']='C'
d1['d']='D'
for k,v in d1.items():
    print(k,v)
if not d1:
    print('no')

d1.pop('b')
print(d1)
print(d1.values())