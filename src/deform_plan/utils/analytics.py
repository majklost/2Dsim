def print_analytics(analytics,steps):
    with open("analytics.txt","w+") as f:
        f.write(get_str_analytics(analytics,steps))
    print(get_str_analytics(analytics,steps))

def get_str_analytics(analytics,steps):
    res = "Analytics: \n"
    total =0
    for k,v in analytics["TIMES"].items():
        res += f"{k}: {v}\n"
        total += v
    res += "Total in check_path: "+str(total)+"\n"
    res += "Outside Check_path:" + str(analytics["OUTSIDE_CHPATH"]) +"\n"
    res += "Average_step_cnt: "+str(analytics["SUM_STEP_CNT"]/steps)+"\n"
    res += "Filled_cnt: "+str(analytics["FILLED_CNT"])+"\n"
    res += "Collided_cnt: "+str(analytics["COLLIDED_CNT"])+"\n"
    res += "Reached_cnt: "+str(analytics["REACHED_CNT"])+"\n"
    res += "Total time taken: "+str(analytics["TOTAL"])+"\n"
    res += "Rejected cnt: "+str(analytics.get("REJECTED_CNT",0))+"\n"
    return res
