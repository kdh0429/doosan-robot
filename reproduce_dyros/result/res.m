clear all;
result=load('result.txt');
for i=1:6
    subplot(2,3,i)
    plot(result(:,[1+i,7+i,13+i]))
    legend('MOB', 'Doosan Friction Model', 'Motor-JTS')
end