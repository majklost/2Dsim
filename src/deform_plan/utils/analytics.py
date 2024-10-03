def print_analytics(analytics,steps):
    with open("analytics.txt","w") as f:
        f.write("Analytics: \n")
        total =0
        for k,v in analytics["TIMES"].items():
            f.write(f"{k}: {v}\n")
            total += v
        f.write("Total in check_path: "+str(total)+"\n")
        f.write("Shit okolo:" + str(analytics["SHITOKOLO"]) +"\n")
        f.write("Average_step_cnt: "+str(analytics["SUM_STEP_CNT"]/steps)+"\n")
        f.write("Filled_cnt: "+str(analytics["FILLED_CNT"])+"\n")
        f.write("Collided_cnt: "+str(analytics["COLLIDED_CNT"])+"\n")
        f.write("Reached_cnt: "+str(analytics["REACHED_CNT"])+"\n")
        f.write("Total time taken: "+str(analytics["TOTAL"])+"\n")
    print("Analytics: ")
    total =0
    for k,v in analytics["TIMES"].items():
        print(f"{k}: {v}")
        total += v
    print("Total: ",total)
    print("Average_step_cnt: ",analytics["SUM_STEP_CNT"]/steps)
    print("Filled_cnt: ",analytics["FILLED_CNT"])
    print("Collided_cnt: ",analytics["COLLIDED_CNT"])
    print("Reached_cnt: ",analytics["REACHED_CNT"])
