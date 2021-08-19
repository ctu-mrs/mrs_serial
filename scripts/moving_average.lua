data_arr = {}
count = 1
flag = false
number = 50

function add (a)
   local sum = 0
     for i,v in ipairs(a) do
        sum = sum + v
     end
     return sum
end



if (flag) then
    s = add(data_arr)
    data_arr[count] = value
    if (count == number) then count = 1 else count = count + 1 end
    return s / (number)
else 
    data_arr[count] = value
    count = count + 1
   
    if (count == number) then
        flag = true
        count = 1
    end
    return value
end
